#include "threads_includes/IR_CaptureThread.hpp"

#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <iostream>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <vector>

#include "util/common_log.hpp"

namespace flir {

namespace { constexpr const char* TAG = "IR_Cap"; }

IR_CaptureThread::IR_CaptureThread(
    std::string name,
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& output_mailbox,
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& out_trk,
    std::unique_ptr<WakeHandle> wake_handle,
    const IRCaptureConfig& config)
    : name_(std::move(name))
    , output_mailbox_(output_mailbox)
    , output_trk_(&out_trk)
    , wake_handle_(std::move(wake_handle))
    , config_(config)
    , spi_fd_(-1)
    , segment_buffer_(vospi::PACKETS_PER_SEGMENT * vospi::PAYLOAD_SIZE)
    , frame_buffer_(vospi::FRAME_WIDTH * vospi::FRAME_HEIGHT)
    , packet_buffer_(vospi::PACKET_SIZE)
{}

IR_CaptureThread::~IR_CaptureThread(){
    stop();
    join();
    cleanup_spi();
}

// -------------------- Reset (safe point only) --------------------

void IR_CaptureThread::perform_safe_reset(){
    std::lock_guard<std::mutex> lock(spi_mutex_);
    
    if (spi_fd_ >= 0) {
        ::close(spi_fd_);
        spi_fd_ = -1;
    }

    // I2C reboot
    int i2c_fd = ::open("/dev/i2c-0", O_RDWR);
    if (i2c_fd >= 0) {
        if (::ioctl(i2c_fd, 0x0703, 0x2a) >= 0) {
            uint8_t reboot_cmd[4] = {0x08, 0x02, 0x00, 0x00};
            if (::write(i2c_fd, reboot_cmd, 4) == 4) {
                ::usleep(750000);
            }
        }
        ::close(i2c_fd);
    }

    reset_requested_.store(false);
    initialize_spi();
}

void IR_CaptureThread::reset_camera(){
    reset_requested_.store(true);
}

// -------------------- Start/Stop/Join --------------------

void IR_CaptureThread::start(){
    if (th_.joinable()) return;

    if (!initialize_spi()) {
        throw std::runtime_error("SPI init failed");
    }

    running_.store(true);
    watchdog_running_.store(true);
    watchdog_thread_ = std::thread(&IR_CaptureThread::watchdog_run, this);
    th_ = std::thread(&IR_CaptureThread::run, this);
}

void IR_CaptureThread::stop(){
    running_.store(false);
    watchdog_running_.store(false);
}

void IR_CaptureThread::join(){
    if (th_.joinable()) th_.join();
    if (watchdog_thread_.joinable()) watchdog_thread_.join();
}

// -------------------- Watchdog --------------------

void IR_CaptureThread::watchdog_run(){
    uint64_t last = 0;
    
    while (watchdog_running_.load()){
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (!running_.load()) continue;
        
        uint64_t cur = frame_count_.load();
        if (cur == last && cur > 0){
            reset_requested_.store(true);
        }
        last = cur;
    }
}

// -------------------- SPI I/O --------------------

bool IR_CaptureThread::initialize_spi() {
    if (spi_fd_ >= 0) {
        ::close(spi_fd_);
        spi_fd_ = -1;
    }

    int fd = ::open(config_.spi_device.c_str(), O_RDWR);
    if (fd < 0) return false;

    uint8_t mode = SPI_MODE_3;
    uint8_t bpw = 8;
    uint32_t spd = config_.spi_speed;

    if (::ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ::ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0 ||
        ::ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spd) < 0) {
        ::close(fd);
        return false;
    }

    spi_fd_ = fd;
    return true;
}

void IR_CaptureThread::cleanup_spi(){
    if (spi_fd_ >= 0) { 
        ::close(spi_fd_); 
        spi_fd_ = -1; 
    }
}

// -------------------- Main loop --------------------

void IR_CaptureThread::run(){
    auto period = std::chrono::microseconds(1'000'000 / std::max(1, config_.fps));
    auto next_t = std::chrono::steady_clock::now();

    while (running_.load()){
        auto frame_t0 = std::chrono::steady_clock::now();

        if (reset_requested_.load()){
            perform_safe_reset();
            next_t = std::chrono::steady_clock::now();
            continue;
        }

        if (capture_vospi_frame()){
            auto h = create_frame_handle();
            if (h){
                push_frame_routed_(h);
                frame_count_.fetch_add(1);
            }
        }

        next_t += period;
        std::this_thread::sleep_until(std::max(next_t, frame_t0 + std::chrono::microseconds(1000)));
    }
}

// -------------------- VoSPI --------------------

bool IR_CaptureThread::capture_vospi_frame(){
    for (int seg=0; seg<vospi::SEGMENTS_PER_FRAME; ++seg){
        if (!capture_segment(seg)) return false;
        if (seg < vospi::SEGMENTS_PER_FRAME-1)
            std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    reconstruct_frame();
    return true;
}

bool IR_CaptureThread::capture_segment(int seg_id){
    const int seg_base = seg_id * (vospi::PACKETS_PER_SEGMENT * vospi::PAYLOAD_SIZE);
    int expected_line = 0;
    int packets = 0;
    int resets = 0;

    while (running_.load() && packets < vospi::PACKETS_PER_SEGMENT) {
        if (!read_vospi_packet(packet_buffer_.data())) return false;

        if (is_discard_packet(packet_buffer_.data())) {
            ::usleep(1000);
            if (++resets > 750) perform_safe_reset();
            continue;
        }

        const int line = get_packet_line_number(packet_buffer_.data());

        if (line >= vospi::LINES_PER_SEGMENT) {
            ::usleep(1000);
            continue;
        }

        if (line != expected_line) {
            expected_line = 0;
            packets = 0;
            ::usleep(1000);
            continue;
        }

        const int off = seg_base + packets * vospi::PAYLOAD_SIZE;
        std::memcpy(&segment_buffer_[off], &packet_buffer_[4], vospi::PAYLOAD_SIZE);

        ++packets;
        ++expected_line;
        resets = 0;
    }

    return (packets == vospi::PACKETS_PER_SEGMENT);
}

bool IR_CaptureThread::read_vospi_packet(uint8_t* pkt){
    std::lock_guard<std::mutex> lock(spi_mutex_);

    if (spi_fd_ < 0) return false;

    struct spi_ioc_transfer tr{};
    tr.tx_buf = 0;
    tr.rx_buf = reinterpret_cast<uintptr_t>(pkt);
    tr.len = vospi::PACKET_SIZE;
    tr.speed_hz = config_.spi_speed;
    tr.bits_per_word = 8;
    tr.cs_change = 1;
    tr.delay_usecs = static_cast<__u16>(config_.spi_delay_usecs);

    return ::ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr) >= 0;
}

void IR_CaptureThread::reconstruct_frame(){
    for (int seg=0; seg<vospi::SEGMENTS_PER_FRAME; ++seg){
        for (int line=0; line<vospi::LINES_PER_SEGMENT; ++line){
            const int frame_line = seg*vospi::LINES_PER_SEGMENT + line;
            const int seg_off = (seg*vospi::PACKETS_PER_SEGMENT*vospi::PAYLOAD_SIZE) + (line*vospi::PAYLOAD_SIZE);
            for (int px=0; px<vospi::PIXELS_PER_LINE; ++px){
                const int frame_idx = frame_line*vospi::FRAME_WIDTH + px;
                const int seg_idx = seg_off + (px*2);
                
                const uint16_t v = (static_cast<uint16_t>(segment_buffer_[seg_idx])<<8)
                                 | static_cast<uint16_t>(segment_buffer_[seg_idx+1]);
                frame_buffer_[frame_idx] = v;
            }
        }
    }
}

// -------------------- Frame handle & routing --------------------

std::shared_ptr<IRFrameHandle> IR_CaptureThread::create_frame_handle(){
    if (frame_buffer_.empty()) return nullptr;
    
    auto handle = std::make_shared<IRMatHandle>();
    
    cv::Mat temp_frame(vospi::FRAME_HEIGHT, vospi::FRAME_WIDTH, CV_16UC1, frame_buffer_.data());
    if (temp_frame.empty()) return nullptr;
    
    handle->keep = std::make_shared<cv::Mat>(temp_frame.clone());
    if (!handle->keep || handle->keep->empty()) return nullptr;
    
    auto& owned = handle->owned;
    owned.data = reinterpret_cast<uint16_t*>(handle->keep->data);
    owned.width = handle->keep->cols;
    owned.height = handle->keep->rows;
    owned.step = static_cast<int>(handle->keep->step);
    
    handle->p = &owned;
    handle->seq = sequence_.fetch_add(1);
    handle->ts = get_timestamp_ns();
    
    return handle;
}

void IR_CaptureThread::push_frame_routed_(const std::shared_ptr<IRFrameHandle>& h){
    // 1) TX로 항상
    output_mailbox_.push(h);
    if (wake_handle_) wake_handle_->signal();

    // 2) 기존 흐름 유지: Terminal일 때만 트래커로 전달
    auto phase = GuidanceState::phase().load(std::memory_order_relaxed);
    const bool route_to_track = (phase == GuidancePhase::Terminal);
    if (!route_to_track) return;

    if (track_sink_) { track_sink_(h); return; }
    if (output_trk_) {
        output_trk_->push(h);
        if (track_wake_) track_wake_->signal();
    }
}

// -------------------- Packet helpers --------------------

inline bool IR_CaptureThread::is_sync_packet(const uint8_t* p){
    return (p[0]==0x00) && (p[1]==0x00);
}

inline bool IR_CaptureThread::is_discard_packet(const uint8_t* p){
    return ((p[0] & 0x0F) == 0x0F);
}

inline int IR_CaptureThread::get_packet_line_number(const uint8_t* p){
    return static_cast<int>(p[1]);
}

uint64_t IR_CaptureThread::get_timestamp_ns(){
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

} // namespace flir

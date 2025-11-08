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

// -------------------- Ctor / Dtor --------------------

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
    // Lepton 2.x 호환: 세그먼트 버퍼는 "세그먼트 1개" 크기로 시작
    // (Lepton 3.x 를 쓸 경우 SEGMENTS_PER_FRAME 값에 맞춰 아래 capture에서 seg_base 오프셋을 합산해 씁니다.)
    , segment_buffer_(vospi::PACKETS_PER_SEGMENT * vospi::PAYLOAD_SIZE)
    , frame_buffer_(vospi::FRAME_WIDTH * vospi::FRAME_HEIGHT)
    , packet_buffer_(vospi::PACKET_SIZE)
{}

IR_CaptureThread::~IR_CaptureThread(){
    stop();
    join();
    cleanup_spi();
}

// -------------------- Reset (안전 지점에서만 수행) --------------------

void IR_CaptureThread::perform_safe_reset(){
    LOGW(TAG, "Resetting camera (safe)...");
    std::lock_guard<std::mutex> lock(spi_mutex_);

    if (spi_fd_ >= 0) {
        ::close(spi_fd_);
        spi_fd_ = -1;
    }

    // I2C reboot (Lepton: /dev/i2c-0, 0x2A)
    int i2c_fd = ::open("/dev/i2c-0", O_RDWR);
    if (i2c_fd >= 0) {
        if (::ioctl(i2c_fd, 0x0703, 0x2a) >= 0) { // I2C_SLAVE
            uint8_t reboot_cmd[4] = {0x08, 0x02, 0x00, 0x00};
            if (::write(i2c_fd, reboot_cmd, 4) == 4) {
                ::usleep(750000); // 750ms
            } else {
                LOGE(TAG, "I2C reboot write failed");
            }
        } else {
            LOGE(TAG, "I2C_SLAVE set failed");
        }
        ::close(i2c_fd);
    } else {
        LOGE(TAG, "open(/dev/i2c-0) failed");
    }

    reset_requested_.store(false);

    if (!initialize_spi()) {
        LOGE(TAG, "Failed to reinitialize SPI after reset");
    }

    reset_cv_.notify_one();
}

void IR_CaptureThread::reset_camera(){
    reset_requested_.store(true);
    std::unique_lock<std::mutex> lock(reset_mutex_);
    reset_cv_.wait_for(lock, std::chrono::seconds(5), [this]() {
        return !reset_requested_.load();
    });
}

// -------------------- Start/Stop/Join --------------------

void IR_CaptureThread::start(){
    if (th_.joinable()) return;

    {
        std::lock_guard<std::mutex> lock(spi_mutex_);
        if (!initialize_spi()) {
            LOGE(TAG, "SPI init failed");
            throw std::runtime_error("IR_CaptureThread failed to initialize SPI");
        }
    }

    running_.store(true);
    watchdog_running_.store(true);
    watchdog_thread_ = std::thread(&IR_CaptureThread::watchdog_run, this);
    th_ = std::thread(&IR_CaptureThread::run, this);
}

void IR_CaptureThread::stop(){
    running_.store(false);
    watchdog_running_.store(false);
    reset_cv_.notify_all();
}

void IR_CaptureThread::join(){
    if (th_.joinable()) th_.join();
    if (watchdog_thread_.joinable()) watchdog_thread_.join();
}

// -------------------- Watchdog --------------------

void IR_CaptureThread::watchdog_run(){
    uint64_t last = 0;
    int stuck_count = 0;

    while (watchdog_running_.load()){
        std::this_thread::sleep_for(std::chrono::seconds(2));
        if (!running_.load()) continue;

        uint64_t cur = frame_count_.load();
        if (cur == last && cur > 0){
            if (++stuck_count >= 3){  // ~6s 정지 감지
                LOGW(TAG, "No new frames ~6s → request reset");
                reset_requested_.store(true);

                std::unique_lock<std::mutex> lock(reset_mutex_);
                reset_cv_.wait_for(lock, std::chrono::seconds(5), [this]() {
                    return !reset_requested_.load();
                });

                stuck_count = 0;
            }
        } else {
            stuck_count = 0;
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
    if (fd < 0) {
        LOGE(TAG, "open SPI %s failed", config_.spi_device.c_str());
        return false;
    }

    uint8_t mode = SPI_MODE_3;  // CPOL=1, CPHA=1
    uint8_t bpw  = 8;
    uint32_t spd = config_.spi_speed;

    if (::ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
        LOGE(TAG, "SPI mode set fail");
        ::close(fd);
        return false;
    }
    if (::ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw) < 0) {
        LOGE(TAG, "SPI bpw set fail");
        ::close(fd);
        return false;
    }
    if (::ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spd) < 0) {
        LOGE(TAG, "SPI speed set fail");
        ::close(fd);
        return false;
    }

    spi_fd_ = fd;

    uint32_t actual = 0;
    if (::ioctl(spi_fd_, SPI_IOC_RD_MAX_SPEED_HZ, &actual) >= 0) {
        LOGI(TAG, "SPI ok: req=%uHz act=%uHz",
             (unsigned)config_.spi_speed, (unsigned)actual);
    } else {
        LOGI(TAG, "SPI ok: req=%uHz", (unsigned)config_.spi_speed);
    }

    return true;
}

void IR_CaptureThread::cleanup_spi(){
    std::lock_guard<std::mutex> lk(spi_mutex_);
    if (spi_fd_ >= 0) { ::close(spi_fd_); spi_fd_ = -1; }
}

// -------------------- Main loop --------------------

void IR_CaptureThread::run(){
    LOGI(TAG, "IR capture thread started.");

    auto period = std::chrono::microseconds(1'000'000 / std::max(1, config_.fps));
    auto next_t = std::chrono::steady_clock::now();

    auto log_t0 = std::chrono::steady_clock::now();
    uint64_t last_f=0, last_e=0, last_d=0;

    // 상수 로깅(디버깅 편의)
    LOGI(TAG, "VoSPI cfg: SEG=%d PPK=%d LINES=%d W=%d H=%d PAYLOAD=%d",
         vospi::SEGMENTS_PER_FRAME, vospi::PACKETS_PER_SEGMENT,
         vospi::LINES_PER_SEGMENT, vospi::FRAME_WIDTH,
         vospi::FRAME_HEIGHT, vospi::PAYLOAD_SIZE);

    while (running_.load()){
        auto frame_t0 = std::chrono::steady_clock::now();

        if (reset_requested_.load()){
            perform_safe_reset();
            next_t = std::chrono::steady_clock::now();
            continue;
        }

        try{
            if (capture_vospi_frame()){
                auto h = create_frame_handle();
                if (h){
                    push_frame_routed_(h);
                    frame_count_.fetch_add(1);
                } else {
                    error_count_.fetch_add(1);
                }
            } else {
                error_count_.fetch_add(1);
            }
        } catch(const std::exception& e){
            LOGE(TAG, "Capture exception: %s", e.what());
            error_count_.fetch_add(1);
        }

        // 1초 통계
        auto now = std::chrono::steady_clock::now();
        if (now - log_t0 >= std::chrono::seconds(1)){
            uint64_t f = frame_count_.load(), e = error_count_.load(), d = discard_count_.load();
            LOGI(TAG, "IR stats: fps=%llu err+=%llu disc+=%llu (total f=%llu e=%llu d=%llu)",
                 (unsigned long long)(f-last_f), (unsigned long long)(e-last_e), (unsigned long long)(d-last_d),
                 (unsigned long long)f, (unsigned long long)e, (unsigned long long)d);
            last_f=f; last_e=e; last_d=d; log_t0=now;
        }

        next_t += period;
        auto sleep_until = std::max(next_t, frame_t0 + std::chrono::microseconds(1000));
        std::this_thread::sleep_until(sleep_until);
    }

    LOGI(TAG, "IR capture thread stopped.");
}

// -------------------- VoSPI (LeptonThread 스타일) --------------------
//
// * 핵심 포인트 *
// - "라인 번호가 기대와 다르면" 세그먼트 처음부터 다시 (resync)
// - "텔레메트리(line >= LINES_PER_SEGMENT)" 는 완전히 건너뛰되,
//   유효라인(0..LINES_PER_SEGMENT-1)을 정확히 PACKETS_PER_SEGMENT개 채웠을 때만 true 리턴
// - Lepton 3.x 대비: seg_id 오프셋(seg_base) 적용
//

bool IR_CaptureThread::capture_vospi_frame(){
    // 프레임 전체를 구성: seg 0..N-1
    for (int seg=0; seg<vospi::SEGMENTS_PER_FRAME; ++seg){
        if (!capture_segment(seg)) return false;
        if (seg < vospi::SEGMENTS_PER_FRAME-1)
            std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    reconstruct_frame();
    return true;
}

bool IR_CaptureThread::capture_segment(int seg_id){
    constexpr int MAX_RESETS = 750;
    int resets = 0;

    // Lepton 3.x 호환: 세그먼트 시작 오프셋(바이트)
    const int seg_base = seg_id * (vospi::PACKETS_PER_SEGMENT * vospi::PAYLOAD_SIZE);

    int expected_line = 0; // 유효 라인 번호(0..59)
    int packets = 0;       // 유효 라인 개수 카운트

    while (running_.load() && packets < vospi::PACKETS_PER_SEGMENT) {
        // 패킷 버퍼 크기 보장
        if (packet_buffer_.size() < vospi::PACKET_SIZE) {
            LOGE(TAG, "CRITICAL: packet buffer too small! size=%zu need=%d",
                 packet_buffer_.size(), vospi::PACKET_SIZE);
            return false;
        }

        if (!read_vospi_packet(packet_buffer_.data())) {
            LOGE(TAG, "SPI read failed");
            return false;
        }

        // 0xFxxx (discard/텔레메트리 헤더) → 완전 스킵 (카운트/expected 변동 없음)
        if (is_discard_packet(packet_buffer_.data())) {
            discard_count_.fetch_add(1);
            ::usleep(1000);
            if (++resets >= MAX_RESETS) {
                LOGW(TAG, "Too many discards → safe reset");
                perform_safe_reset();
                resets = 0;
            }
            continue;
        }

        const int line = get_packet_line_number(packet_buffer_.data());

        // 텔레메트리 라인(line >= LINES_PER_SEGMENT) → 완전 스킵
        if (line >= vospi::LINES_PER_SEGMENT) {
            ::usleep(1000);
            continue;
        }

        // 라인 번호가 비정상/역행/불연속 → 세그먼트 재동기
        if (line < 0 || line != expected_line) {
            expected_line = 0;
            packets = 0;
            ::usleep(1000);
            if (++resets >= MAX_RESETS) {
                LOGW(TAG, "Too many resyncs → safe reset");
                perform_safe_reset();
                resets = 0;
            }
            continue;
        }

        // 안전 memcpy
        const int off = seg_base + packets * vospi::PAYLOAD_SIZE;
        if (off < 0 ||
            off + vospi::PAYLOAD_SIZE > static_cast<int>(segment_buffer_.size())) {
            LOGE(TAG, "CRITICAL: segment buffer memcpy overflow! seg=%d off=%d payload=%d size=%zu",
                 seg_id, off, vospi::PAYLOAD_SIZE, segment_buffer_.size());
            return false;
        }
        if (packet_buffer_.size() < 4 + vospi::PAYLOAD_SIZE) {
            LOGE(TAG, "CRITICAL: packet buffer underflow! size=%zu need=%d",
                 packet_buffer_.size(), 4 + vospi::PAYLOAD_SIZE);
            return false;
        }

        std::memcpy(&segment_buffer_[off], &packet_buffer_[4], vospi::PAYLOAD_SIZE);

        ++packets;        // 유효 라인 누적
        ++expected_line;  // 다음 기대 라인
        resets = 0;
    }

    return (packets == vospi::PACKETS_PER_SEGMENT);
}

bool IR_CaptureThread::read_vospi_packet(uint8_t* pkt){
    std::lock_guard<std::mutex> lock(spi_mutex_);

    if (spi_fd_ < 0){
        LOGE(TAG, "SPI fd invalid");
        return false;
    }
    if (!pkt) {
        LOGE(TAG, "CRITICAL: NULL packet buffer passed to read_vospi_packet");
        return false;
    }

    struct spi_ioc_transfer tr{};
    tr.tx_buf = 0;
    tr.rx_buf = reinterpret_cast<uintptr_t>(pkt);
    tr.len = vospi::PACKET_SIZE;
    tr.speed_hz = config_.spi_speed;
    tr.bits_per_word = 8;
    tr.cs_change = 1;
    tr.delay_usecs = static_cast<__u16>(config_.spi_delay_usecs);

    int r = ::ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    if (r < 0) {
        LOGE(TAG, "SPI ioctl failed: errno=%d", errno);
        return false;
    }
    return true;
}

// 패킷→프레임(16U) 재구성
void IR_CaptureThread::reconstruct_frame(){
    for (int seg=0; seg<vospi::SEGMENTS_PER_FRAME; ++seg){
        for (int line=0; line<vospi::LINES_PER_SEGMENT; ++line){
            const int frame_line = seg*vospi::LINES_PER_SEGMENT + line;
            const int seg_off = (seg*vospi::PACKETS_PER_SEGMENT*vospi::PAYLOAD_SIZE)
                              + (line*vospi::PAYLOAD_SIZE);
            for (int px=0; px<vospi::PIXELS_PER_LINE; ++px){
                const int frame_idx = frame_line*vospi::FRAME_WIDTH + px;
                const int seg_idx   = seg_off + (px*2);

                if (seg_idx + 1 >= static_cast<int>(segment_buffer_.size())) {
                    LOGE(TAG, "CRITICAL: segment buffer overflow! seg_idx=%d+1 size=%zu seg=%d line=%d px=%d",
                         seg_idx, segment_buffer_.size(), seg, line, px);
                    return;
                }
                if (frame_idx >= static_cast<int>(frame_buffer_.size())) {
                    LOGE(TAG, "CRITICAL: frame buffer overflow! frame_idx=%d size=%zu frame_line=%d px=%d",
                         frame_idx, frame_buffer_.size(), frame_line, px);
                    return;
                }

                const uint16_t v = (static_cast<uint16_t>(segment_buffer_[seg_idx])<<8)
                                 | static_cast<uint16_t>(segment_buffer_[seg_idx+1]);
                frame_buffer_[frame_idx] = v;
            }
        }
    }
}

// -------------------- Frame handle & routing --------------------

std::shared_ptr<IRFrameHandle> IR_CaptureThread::create_frame_handle(){
    if (frame_buffer_.empty()) {
        return nullptr;
    }

    auto handle = std::make_shared<IRMatHandle>();

    cv::Mat temp_frame(vospi::FRAME_HEIGHT, vospi::FRAME_WIDTH, CV_16UC1, frame_buffer_.data());
    if (temp_frame.empty()) {
        return nullptr;
    }

    handle->keep = std::make_shared<cv::Mat>(temp_frame.clone());
    if (!handle->keep || handle->keep->empty() || !handle->keep->isContinuous()) {
        return nullptr;
    }
    if (!handle->keep->data) {
        return nullptr;
    }

    auto& owned = handle->owned;
    owned.data   = reinterpret_cast<uint16_t*>(handle->keep->data);
    owned.width  = handle->keep->cols;
    owned.height = handle->keep->rows;
    owned.step   = static_cast<int>(handle->keep->step);

    if (!owned.data) {
        return nullptr;
    }
    if (owned.width != vospi::FRAME_WIDTH || owned.height != vospi::FRAME_HEIGHT) {
        return nullptr;
    }

    handle->p   = &owned;
    handle->seq = sequence_.fetch_add(1);
    handle->ts  = get_timestamp_ns();
    return handle;
}

void IR_CaptureThread::push_frame_routed_(const std::shared_ptr<IRFrameHandle>& h){
    // TX로 항상
    output_mailbox_.push(h);
    if (wake_handle_) wake_handle_->signal();

    // Terminal일 때만 트래커로 라우팅
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

// 0xFxxx 헤더(텔레메트리/디스카드) 검출
inline bool IR_CaptureThread::is_discard_packet(const uint8_t* p){
    return ((p[0] & 0x0F) == 0x0F);
}

// 두 번째 바이트(라인 번호)
inline int IR_CaptureThread::get_packet_line_number(const uint8_t* p){
    return static_cast<int>(p[1]);
}

uint64_t IR_CaptureThread::get_timestamp_ns(){
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

} // namespace flir

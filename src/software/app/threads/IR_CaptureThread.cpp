#include "threads_includes/IR_CaptureThread.hpp"

#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

// ★ 공용 로거
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
{}

IR_CaptureThread::~IR_CaptureThread() {
    stop();
    join();
    cleanup_spi();
}

// Power cycle Lepton by closing and reopening SPI
void IR_CaptureThread::reset_camera() {
    LOGW(TAG, "Resetting camera...");

    if (spi_fd_ >= 0) {
        close(spi_fd_);
        spi_fd_ = -1;
    }

    // I2C reboot (Lepton at /dev/i2c-0, addr 0x2a)
    int i2c_fd = open("/dev/i2c-0", O_RDWR);
    if (i2c_fd >= 0) {
        if (ioctl(i2c_fd, 0x0703, 0x2a) >= 0) { // I2C_SLAVE
            uint8_t reboot_cmd[4] = {0x08, 0x02, 0x00, 0x00};
            ssize_t written = write(i2c_fd, reboot_cmd, 4);
            if (written == 4) {
                LOGI(TAG, "Sent Lepton reboot command via I2C (0x2a).");
                usleep(750000); // ~750ms
            } else {
                LOGE(TAG, "Failed to send Lepton reboot command via I2C.");
            }
        } else {
            LOGE(TAG, "Failed to set I2C_SLAVE for Lepton.");
        }
        close(i2c_fd);
    } else {
        LOGE(TAG, "Failed to open /dev/i2c-0 for Lepton reboot.");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (!initialize_spi()) {
        LOGE(TAG, "Failed to reinitialize SPI after reset.");
    }
}

void IR_CaptureThread::start() {
    if (th_.joinable()) return;

    if (!initialize_spi()) {
        LOGE(TAG, "SPI init failed.");
        throw std::runtime_error("IR_CaptureThread failed to initialize SPI");
    }

    running_.store(true);
    watchdog_running_.store(true);
    watchdog_thread_ = std::thread(&IR_CaptureThread::watchdog_run, this);

    th_ = std::thread(&IR_CaptureThread::run, this);
}

void IR_CaptureThread::stop() {
    running_.store(false);
    watchdog_running_.store(false);
}

void IR_CaptureThread::join() {
    if (th_.joinable()) th_.join();
    if (watchdog_thread_.joinable()) watchdog_thread_.join();
}

// Watchdog: 1초 간격 점검, 2초 동안 신프레임 없으면 reset
void IR_CaptureThread::watchdog_run() {
    uint64_t last_frame_count = 0;
    int no_new_secs = 0;

    while (watchdog_running_.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        uint64_t cur = frame_count_.load();

        if (cur == last_frame_count && cur > 0) {
            ++no_new_secs;
            if (no_new_secs >= 2) { // 2s stuck
                LOGW(TAG, "No new frames for 2s → resetting camera...");
                reset_camera();
                no_new_secs = 0;
            }
        } else {
            no_new_secs = 0;
        }
        last_frame_count = cur;
    }
}

bool IR_CaptureThread::initialize_spi() {
    spi_fd_ = open(config_.spi_device.c_str(), O_RDWR);
    if (spi_fd_ < 0) {
        LOGE(TAG, "Failed to open SPI device: %s", config_.spi_device.c_str());
        return false;
    }

    uint8_t spi_mode = SPI_MODE_3; // MSB-first (default)
    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &spi_mode) < 0) {
        LOGE(TAG, "Failed to set SPI mode.");
        close(spi_fd_); spi_fd_ = -1; return false;
    }

    uint8_t bits_per_word = 8;
    if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
        LOGE(TAG, "Failed to set SPI bits per word.");
        close(spi_fd_); spi_fd_ = -1; return false;
    }

    uint32_t spi_speed = config_.spi_speed;
    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) {
        LOGE(TAG, "Failed to set SPI speed.");
        close(spi_fd_); spi_fd_ = -1; return false;
    }

    uint32_t actual_speed = 0;
    if (ioctl(spi_fd_, SPI_IOC_RD_MAX_SPEED_HZ, &actual_speed) >= 0) {
        LOGI(TAG, "SPI initialized: %s @ %u Hz (requested), %u Hz (actual)",
             config_.spi_device.c_str(), (unsigned)spi_speed, (unsigned)actual_speed);
    } else {
        LOGI(TAG, "SPI initialized: %s @ %u Hz",
             config_.spi_device.c_str(), (unsigned)spi_speed);
    }
    return true;
}

void IR_CaptureThread::cleanup_spi() {
    if (spi_fd_ >= 0) { close(spi_fd_); spi_fd_ = -1; }
}

void IR_CaptureThread::push_frame_routed_(const std::shared_ptr<IRFrameHandle>& h) {
    // TX 경로 (기존)
    output_mailbox_.push(h);
    if (wake_handle_) wake_handle_->signal();

    // 단계 기반 (기존: Terminal에서만 트랙)
    auto phase = GuidanceState::phase().load(std::memory_order_relaxed);
    const bool route_to_track = (phase == GuidancePhase::Terminal);
    if (!route_to_track) return;

    if (track_sink_) {
        track_sink_(h);                          // onFrameArrived() 경유 → 내부에서 notify
        return;
    }
    if (output_trk_) {
        output_trk_->push(h);                    // 메일박스 경로
        if (track_wake_) track_wake_->signal();  // ★ 직접 깨움
    }
}

void IR_CaptureThread::run() {
    LOGI(TAG, "IR capture thread started.");

    auto target_period = std::chrono::microseconds(1'000'000 / std::max(1, config_.fps));
    auto next_frame_time = std::chrono::steady_clock::now();

    // 1초 주기 처리 로그
    auto log_t0 = std::chrono::steady_clock::now();
    uint64_t last_f = 0, last_e = 0, last_d = 0;

    while (running_.load()) {
        auto frame_start = std::chrono::steady_clock::now();

        try {
            if (capture_vospi_frame()) {
                auto frame = create_frame_handle();
                if (frame) {
                    push_frame_routed_(frame);
                    frame_count_.fetch_add(1);
                }
            } else {
                error_count_.fetch_add(1);
            }
        } catch (const std::exception& e) {
            LOGE(TAG, "Capture exception: %s", e.what());
            error_count_.fetch_add(1);
        }

        // — 1초 주기 처리 로그 —
        auto now = std::chrono::steady_clock::now();
        if (now - log_t0 >= std::chrono::seconds(1)) {
            uint64_t f = frame_count_.load();
            uint64_t e = error_count_.load();
            uint64_t d = discard_count_.load();
            LOGI(TAG, "IR stats: fps=%llu err+=%llu disc+=%llu (total f=%llu, e=%llu, d=%llu)",
                 (unsigned long long)(f - last_f),
                 (unsigned long long)(e - last_e),
                 (unsigned long long)(d - last_d),
                 (unsigned long long)f,
                 (unsigned long long)e,
                 (unsigned long long)d);
            last_f = f; last_e = e; last_d = d;
            log_t0 = now;
        }

        next_frame_time += target_period;
        auto sleep_until = std::max(next_frame_time, frame_start + std::chrono::microseconds(1000));
        std::this_thread::sleep_until(sleep_until);
    }

    LOGI(TAG, "IR capture thread stopped. Frames=%llu Errors=%llu Discards=%llu",
         (unsigned long long)frame_count_.load(),
         (unsigned long long)error_count_.load(),
         (unsigned long long)discard_count_.load());
}

bool IR_CaptureThread::capture_vospi_frame() {
    for (int segment = 0; segment < vospi::SEGMENTS_PER_FRAME; segment++) {
        if (!capture_segment(segment)) return false;
        if (segment < vospi::SEGMENTS_PER_FRAME - 1)
            std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
    reconstruct_frame();
    return true;
}

bool IR_CaptureThread::capture_segment(int) {
    uint8_t packet_buffer[vospi::PACKET_SIZE];
    int resets = 0;
    const int MAX_RESETS = 750;

    while (running_.load()) {
        int packets_received = 0;

        for (int expected_line = 0; expected_line < vospi::PACKETS_PER_SEGMENT; expected_line++) {
            if (!read_vospi_packet(packet_buffer)) {
                return false;
            }

            if (is_discard_packet(packet_buffer)) {
                discard_count_.fetch_add(1);
                usleep(1000);
                expected_line = -1;
                if (++resets >= MAX_RESETS) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(185));
                    resets = 0;
                }
                continue;
            }

            int line_number = get_packet_line_number(packet_buffer);
            if (line_number < 0 || line_number >= vospi::LINES_PER_SEGMENT) {
                expected_line = -1;
                usleep(1000);
                if (++resets >= MAX_RESETS) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(185));
                    resets = 0;
                }
                continue;
            }

            if (line_number != expected_line) {
                expected_line = -1;
                usleep(1000);
                if (++resets >= MAX_RESETS) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(185));
                    resets = 0;
                }
                continue;
            }

            int buffer_offset = packets_received * vospi::PAYLOAD_SIZE;
            std::memcpy(&segment_buffer_[buffer_offset], &packet_buffer[4], vospi::PAYLOAD_SIZE);
            ++packets_received;
            resets = 0;
        }

        if (packets_received == vospi::PACKETS_PER_SEGMENT) return true;
    }
    return false;
}

bool IR_CaptureThread::read_vospi_packet(uint8_t* packet_buffer) {
    struct spi_ioc_transfer transfer = {};
    transfer.tx_buf = 0;
    transfer.rx_buf = reinterpret_cast<uintptr_t>(packet_buffer);
    transfer.len = vospi::PACKET_SIZE;
    transfer.speed_hz = config_.spi_speed;
    transfer.bits_per_word = 8;
    transfer.cs_change = 1;
    transfer.delay_usecs = static_cast<__u16>(config_.spi_delay_usecs);

    int result = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &transfer);
    return result >= 0;
}

void IR_CaptureThread::reconstruct_frame() {
    for (int segment = 0; segment < vospi::SEGMENTS_PER_FRAME; segment++) {
        for (int line = 0; line < vospi::LINES_PER_SEGMENT; line++) {
            int frame_line = segment * vospi::LINES_PER_SEGMENT + line;
            int segment_offset = (segment * vospi::PACKETS_PER_SEGMENT * vospi::PAYLOAD_SIZE) +
                                 (line * vospi::PAYLOAD_SIZE);

            for (int pixel = 0; pixel < vospi::PIXELS_PER_LINE; pixel++) {
                int frame_pixel_idx = frame_line * vospi::FRAME_WIDTH + pixel;
                int segment_byte_idx = segment_offset + (pixel * 2);
                uint16_t pixel_value = (static_cast<uint16_t>(segment_buffer_[segment_byte_idx]) << 8) |
                                       static_cast<uint16_t>(segment_buffer_[segment_byte_idx + 1]);
                frame_buffer_[frame_pixel_idx] = pixel_value;
            }
        }
    }
}

std::shared_ptr<IRFrameHandle> IR_CaptureThread::create_frame_handle() {
    cv::Mat thermal_frame(vospi::FRAME_HEIGHT, vospi::FRAME_WIDTH, CV_16UC1, frame_buffer_.data());
    auto handle = std::make_shared<IRMatHandle>();
    handle->keep = std::make_shared<cv::Mat>(thermal_frame.clone());

    auto& owned = handle->owned;
    owned.data = reinterpret_cast<uint16_t*>(handle->keep->data);
    owned.width = handle->keep->cols;
    owned.height = handle->keep->rows;
    owned.step = static_cast<int>(handle->keep->step);

    handle->seq = sequence_.fetch_add(1);
    handle->ts = get_timestamp_ns();
    return handle;
}

bool IR_CaptureThread::is_sync_packet(const uint8_t* packet) {
    return (packet[0] == 0x00) && (packet[1] == 0x00);
}

bool IR_CaptureThread::is_discard_packet(const uint8_t* packet) {
    return ((packet[0] & 0x0F) == 0x0F);
}

int IR_CaptureThread::get_packet_line_number(const uint8_t* packet) {
    return static_cast<int>(packet[1]);
}

uint64_t IR_CaptureThread::get_timestamp_ns() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

} // namespace flir

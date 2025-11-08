#pragma once
#include <atomic>
#include <thread>
#include <string>
#include <memory>
#include <vector>
#include <chrono>
#include <functional>
#include <condition_variable>
#include <mutex>

#include <opencv2/core.hpp>
#include "components/includes/IR_Frame.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/wake.hpp"
#include "guidance_mode.hpp"

namespace flir {

// VoSPI (Video over SPI) protocol constants for FLIR Lepton 2.5
namespace vospi {
    constexpr int PACKET_SIZE = 164;
    constexpr int PAYLOAD_SIZE = 160;
    constexpr int PIXELS_PER_LINE = 80;
    constexpr int LINES_PER_SEGMENT = 60;
    constexpr int PACKETS_PER_SEGMENT = 60;
    constexpr int SEGMENTS_PER_FRAME = 1;
    constexpr int FRAME_WIDTH = 80;
    constexpr int FRAME_HEIGHT = 60;
}

struct IRCaptureConfig {
    std::string spi_device = "/dev/spidev1.0";
    uint32_t spi_speed = 2'000'000;
    int fps = 9;
    // inter-transfer delay between chip-select toggles (µs)
    uint32_t spi_delay_usecs = 50;
};

struct IRMatHandle : IRFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    IRFrame16 owned{};
    IRMatHandle() { p = &owned; }
    ~IRMatHandle() override = default;
    void retain() override {}
    void release() override {}
};

class IR_CaptureThread {
public:
    IR_CaptureThread(
        std::string name,
        SpscMailbox<std::shared_ptr<IRFrameHandle>>& out_tx,
        SpscMailbox<std::shared_ptr<IRFrameHandle>>& out_trk,   // ★ 추가
        std::unique_ptr<WakeHandle> wake_tx,                    // TX 깨우기 (EO와 동일 정책)
        const IRCaptureConfig& config = IRCaptureConfig{}
    );
    ~IR_CaptureThread();

    void start();
    void stop();
    void join();

    // 추적 쪽으로 직접 전달(싱크)하거나, 별도 메일박스로 푸시할 수 있도록 사후 주입
    void set_track_sink(std::function<void(std::shared_ptr<IRFrameHandle>)> sink) noexcept {
        track_sink_ = std::move(sink);
    }
    void set_track_mailbox(SpscMailbox<std::shared_ptr<IRFrameHandle>>* mb) noexcept {
        output_trk_ = mb;
    }

    // Statistics
    uint64_t get_frame_count() const { return frame_count_.load(); }
    uint64_t get_error_count() const { return error_count_.load(); }
    uint64_t get_discard_count() const { return discard_count_.load(); }

    void reset_camera();
    void set_track_wake(std::unique_ptr<WakeHandle> w) noexcept { track_wake_ = std::move(w); } // ★ 추가

private:
    std::string name_;
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& output_mailbox_; // TX
    SpscMailbox<std::shared_ptr<IRFrameHandle>>* output_trk_{nullptr}; // TRACK (옵션)
    std::unique_ptr<WakeHandle> wake_handle_;
    IRCaptureConfig config_;

    std::thread th_;
    std::atomic<bool> running_{false};

    int spi_fd_;
    std::mutex spi_mutex_;  // Protects SPI operations from concurrent access

    // Safe reset mechanism
    std::atomic<bool> reset_requested_{false};
    std::mutex reset_mutex_;
    std::condition_variable reset_cv_;
    void perform_safe_reset();  // Only called by capture thread at safe points

    std::vector<uint8_t>  segment_buffer_;
    std::vector<uint16_t> frame_buffer_;
    std::vector<uint8_t> packet_buffer_;       // Heap-allocated packet buffer (moved from stack to prevent overflow)

    std::atomic<uint64_t> frame_count_{0};
    std::atomic<uint64_t> error_count_{0};
    std::atomic<uint64_t> discard_count_{0};
    std::atomic<uint32_t> sequence_{0};

    std::thread watchdog_thread_;
    std::atomic<bool> watchdog_running_{false};
    void watchdog_run();

    void run();
    bool initialize_spi();
    void cleanup_spi();

    bool capture_vospi_frame();
    bool capture_segment(int segment_id);
    bool read_vospi_packet(uint8_t* packet_buffer);
    void reconstruct_frame();

    bool is_sync_packet(const uint8_t* packet);
    bool is_discard_packet(const uint8_t* packet);
    int  get_packet_line_number(const uint8_t* packet);

    std::shared_ptr<IRFrameHandle> create_frame_handle();
    uint64_t get_timestamp_ns();

    std::function<void(std::shared_ptr<IRFrameHandle>)> track_sink_;

    // 단계 기반 라우팅 (TX는 항상, TRACK은 Terminal에서만)
    void push_frame_routed_(const std::shared_ptr<IRFrameHandle>& h);
    std::unique_ptr<WakeHandle> track_wake_;   // ★ 추가
};

} // namespace flir

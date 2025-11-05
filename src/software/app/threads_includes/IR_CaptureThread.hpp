#pragma once
#include <atomic>
#include <thread>
#include <string>
#include <memory>
#include <vector>
#include <chrono>
#include <functional>

#include <opencv2/core.hpp>
#include "components/includes/IR_Frame.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/wake.hpp"
#include "guidance_mode.hpp"

namespace flir {

// VoSPI (Video over SPI) protocol constants for FLIR Lepton 2.5
namespace vospi {
    constexpr int PACKET_SIZE = 164;           // Total packet size in bytes
    constexpr int PAYLOAD_SIZE = 160;          // Payload size per packet (80 pixels × 2 bytes)
    constexpr int PIXELS_PER_LINE = 80;        // Pixels per line in raw segment
    constexpr int LINES_PER_SEGMENT = 60;      // Lines per segment
    constexpr int PACKETS_PER_SEGMENT = 60;    // Packets per segment (1 packet per line)
    constexpr int SEGMENTS_PER_FRAME = 1;      // Total segments per frame (Lepton 2.5 uses 1 segment)
    constexpr int FRAME_WIDTH = 80;            // Final frame width (Lepton 2.5)
    constexpr int FRAME_HEIGHT = 60;           // Final frame height (Lepton 2.5)
}

struct IRCaptureConfig {
    std::string spi_device = "/dev/spidev1.0";  // SPI device path
    uint32_t spi_speed = 12500000;              // SPI speed in Hz (12.5MHz for stability)
    int fps = 9;                                // Target frame rate (Lepton 3.0 max ~9 fps)
    // Microsecond delay after each SPI transfer. 
    // inter-transfer delay to let the Lepton's VoSPI interface stabilize
    // between chip-select toggles. 
    uint32_t spi_delay_usecs = 50;              // Default 50 µs
};

// MatHandle implementation for IR frames
struct IRMatHandle : IRFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    IRFrame16 owned{};
    IRMatHandle() { p = &owned; }
    
    ~IRMatHandle() override = default;
    
    void retain() override {
        // shared_ptr handles reference counting
    }
    
    void release() override {
        // shared_ptr handles reference counting  
    }
};

class IR_CaptureThread {
public:
    IR_CaptureThread(
        std::string name,
        SpscMailbox<std::shared_ptr<IRFrameHandle>>& output_mailbox,
        std::unique_ptr<WakeHandle> wake_handle,
        const IRCaptureConfig& config = IRCaptureConfig{}
    );
    ~IR_CaptureThread();

    void start();
    void stop();
    void join();

    // ★ 추가: EO처럼 트래킹 큐도 받도록 "사후 주입" (메인 최소 변경)
    void set_track_sink(std::function<void(std::shared_ptr<IRFrameHandle>)> sink) noexcept {
        track_sink_ = std::move(sink);
    }
    // (메일박스로 보내고 싶을 때를 대비해 set_track_mailbox도 유지 가능)
    void set_track_mailbox(SpscMailbox<std::shared_ptr<IRFrameHandle>>* mb) noexcept {
        output_trk_ = mb;
    }

    // Statistics (기존)
    uint64_t get_frame_count() const { return frame_count_.load(); }
    uint64_t get_error_count() const { return error_count_.load(); }
    uint64_t get_discard_count() const { return discard_count_.load(); }

    void reset_camera();

private:
    std::string name_;
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& output_mailbox_;          // TX용 (기존)
    SpscMailbox<std::shared_ptr<IRFrameHandle>>* output_trk_{nullptr};     // ★ 추가: 트랙용(옵션)
    std::unique_ptr<WakeHandle> wake_handle_;
    IRCaptureConfig config_;

    std::thread th_;
    std::atomic<bool> running_{false};

    int spi_fd_;

    std::vector<uint8_t>  segment_buffer_;
    std::vector<uint16_t> frame_buffer_;

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
    // ★ 추가: 단계 기반 라우팅 (EO의 push_frame_과 유사 컨셉)
    void push_frame_routed_(const std::shared_ptr<IRFrameHandle>& h);
};

} // namespace flir
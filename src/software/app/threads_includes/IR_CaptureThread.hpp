#pragma once
#include <atomic>
#include <thread>
#include <string>
#include <memory>
#include <vector>
#include <chrono>

#include <opencv2/core.hpp>
#include "components/includes/IR_Frame.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/wake.hpp"
#include "main_config.hpp"   // AppConfig / AppConfigPtr

namespace flir {

// ===== VoSPI constants (Lepton 2.5) =====
namespace vospi {
    constexpr int PACKET_SIZE         = 164;   // 4B header + 160B payload
    constexpr int PAYLOAD_SIZE        = 160;   // 80 px * 2 B
    constexpr int PIXELS_PER_LINE     = 80;
    constexpr int LINES_PER_SEGMENT   = 60;
    constexpr int PACKETS_PER_SEGMENT = 60;    // 1 packet per line
    constexpr int SEGMENTS_PER_FRAME  = 1;     // Lepton 2.5 = 1
    constexpr int FRAME_WIDTH         = 80;
    constexpr int FRAME_HEIGHT        = 60;
}

// cv::Mat을 보유하는 IR 핸들 (소유권 보장)
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
    // 팬아웃 내장: 캡처 → [out_tx, out_trk] 두 큐로 복제
    IR_CaptureThread(
        std::string name,
        SpscMailbox<std::shared_ptr<IRFrameHandle>>& out_tx,
        SpscMailbox<std::shared_ptr<IRFrameHandle>>& out_trk,
        AppConfigPtr cfg
    );
    ~IR_CaptureThread();

    void start();
    void stop();
    void join();

    // Stats
    uint64_t frames()   const { return frame_count_.load(); }
    uint64_t errors()   const { return error_count_.load(); }
    uint64_t discards() const { return discard_count_.load(); }

    // I2C를 통한 Lepton 재부팅(필요시)
    void reset_camera();

private:
    // threads
    void run();

    // SPI
    bool initialize_spi();
    void cleanup_spi();

    // VoSPI
    bool capture_vospi_frame();
    bool capture_segment(int seg_id);
    bool read_vospi_packet(uint8_t* packet_buffer);
    void reconstruct_frame();

    // Packet helpers
    bool is_discard_packet(const uint8_t* packet);
    int  get_packet_line_number(const uint8_t* packet);

    // Handle
    std::shared_ptr<IRFrameHandle> create_frame_handle();
    uint64_t now_ns();

private:
    std::string name_;
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& out_tx_;
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& out_trk_;
    AppConfigPtr cfg_;

    std::thread th_;
    std::atomic<bool> running_{false};

    // SPI fd
    int spi_fd_{-1};

    // Buffers
    std::vector<uint8_t>  segment_buffer_; // 60 * 160 bytes
    std::vector<uint16_t> frame_buffer_;   // 80 * 60 uint16

    // stats
    std::atomic<uint64_t> frame_count_{0};
    std::atomic<uint64_t> error_count_{0};
    std::atomic<uint64_t> discard_count_{0};
    std::atomic<uint32_t> seq_{0};
};

} // namespace flir

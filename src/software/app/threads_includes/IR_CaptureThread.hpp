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
    uint32_t spi_speed = 10000000;              // SPI speed in Hz (10MHz for stability)
    int fps = 9;                                // Target frame rate (Lepton 3.0 max ~9 fps)
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
    
    // Statistics
    uint64_t get_frame_count() const { return frame_count_.load(); }
    uint64_t get_error_count() const { return error_count_.load(); }
    uint64_t get_discard_count() const { return discard_count_.load(); }
    
private:
    std::string name_;
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& output_mailbox_;
    std::unique_ptr<WakeHandle> wake_handle_;
    IRCaptureConfig config_;
    
    // Thread management
    std::thread th_;
    std::atomic<bool> running_{false};
    
    // SPI interface
    int spi_fd_;
    
    // VoSPI buffers
    std::vector<uint8_t> segment_buffer_;      // Buffer for segment data
    std::vector<uint16_t> frame_buffer_;       // Buffer for reconstructed frame
    
    // Statistics
    std::atomic<uint64_t> frame_count_{0};
    std::atomic<uint64_t> error_count_{0};
    std::atomic<uint64_t> discard_count_{0};
    std::atomic<uint32_t> sequence_{0};
    
    // Internal methods
    void run();
    bool initialize_spi();
    void cleanup_spi();
    
    // VoSPI protocol handling
    bool capture_vospi_frame();
    bool capture_segment(int segment_id);
    bool read_vospi_packet(uint8_t* packet_buffer);
    void reconstruct_frame();
    
    // VoSPI packet analysis
    bool is_sync_packet(const uint8_t* packet);
    bool is_discard_packet(const uint8_t* packet);
    int get_packet_line_number(const uint8_t* packet);
    
    // Frame creation
    std::shared_ptr<IRFrameHandle> create_frame_handle();
    uint64_t get_timestamp_ns();
};

} // namespace flir

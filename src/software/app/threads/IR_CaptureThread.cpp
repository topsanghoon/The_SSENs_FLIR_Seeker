#include "threads_includes/IR_CaptureThread.hpp"
#include <iostream>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

namespace flir {

IR_CaptureThread::IR_CaptureThread(
    std::string name,
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& output_mailbox,
    std::unique_ptr<WakeHandle> wake_handle,
    const IRCaptureConfig& config)
    : name_(std::move(name))
    , output_mailbox_(output_mailbox)
    , wake_handle_(std::move(wake_handle))
    , config_(config)
    , spi_fd_(-1)
    , segment_buffer_(vospi::PACKETS_PER_SEGMENT * vospi::PAYLOAD_SIZE)
    , frame_buffer_(vospi::FRAME_WIDTH * vospi::FRAME_HEIGHT)
{
}

IR_CaptureThread::~IR_CaptureThread() {
    stop();
    join();
    cleanup_spi();
}

void IR_CaptureThread::start() {
    if (th_.joinable()) return;
    
    if (!initialize_spi()) {
        throw std::runtime_error("IR_CaptureThread failed to initialize SPI");
    }
    
    running_.store(true);
    th_ = std::thread(&IR_CaptureThread::run, this);
}

void IR_CaptureThread::stop() {
    running_.store(false);
}

void IR_CaptureThread::join() {
    if (th_.joinable()) {
        th_.join();
    }
}

bool IR_CaptureThread::initialize_spi() {
    // Open SPI device
    spi_fd_ = open(config_.spi_device.c_str(), O_RDWR);
    if (spi_fd_ < 0) {
        std::cerr << "[" << name_ << "] Failed to open SPI device: " << config_.spi_device << std::endl;
        return false;
    }
    
    // Set SPI mode (Mode 3: CPOL=1, CPHA=1)
    // IMPORTANT: SPI_LSB_FIRST must NOT be set - VoSPI uses MSB first (default)
    uint8_t spi_mode = SPI_MODE_3;
    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &spi_mode) < 0) {
        std::cerr << "[" << name_ << "] Failed to set SPI mode" << std::endl;
        close(spi_fd_);
        return false;
    }
    
    // Read back the mode to verify
    uint8_t mode_check = 0;
    if (ioctl(spi_fd_, SPI_IOC_RD_MODE, &mode_check) >= 0) {
        std::cout << "[" << name_ << "] SPI Mode: 0x" << std::hex << (int)mode_check << std::dec << std::endl;
    }
    
    // Set bits per word (8 bits)
    uint8_t bits_per_word = 8;
    if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
        std::cerr << "[" << name_ << "] Failed to set SPI bits per word" << std::endl;
        close(spi_fd_);
        return false;
    }
    
    // Set SPI speed
    uint32_t spi_speed = config_.spi_speed;
    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0) {
        std::cerr << "[" << name_ << "] Failed to set SPI speed" << std::endl;
        close(spi_fd_);
        return false;
    }
    
    // Read back actual speed to verify (hardware may limit it)
    uint32_t actual_speed = 0;
    if (ioctl(spi_fd_, SPI_IOC_RD_MAX_SPEED_HZ, &actual_speed) >= 0) {
        std::cout << "[" << name_ << "] SPI initialized: " << config_.spi_device 
                  << " @ " << spi_speed << " Hz (requested), " 
                  << actual_speed << " Hz (actual)" << std::endl;
    } else {
        std::cout << "[" << name_ << "] SPI initialized: " << config_.spi_device 
                  << " @ " << spi_speed << " Hz" << std::endl;
    }
    
    return true;
}

void IR_CaptureThread::cleanup_spi() {
    if (spi_fd_ >= 0) {
        close(spi_fd_);
        spi_fd_ = -1;
    }
}

void IR_CaptureThread::run() {
    std::cout << "[" << name_ << "] IR capture thread started" << std::endl;
    
    auto target_period = std::chrono::microseconds(1000000 / config_.fps);
    auto next_frame_time = std::chrono::steady_clock::now();
    
    while (running_.load()) {
        auto frame_start = std::chrono::steady_clock::now();
        
        try {
            if (capture_vospi_frame()) {
                auto frame = create_frame_handle();
                if (frame) {
                    output_mailbox_.push(frame);
                    frame_count_.fetch_add(1);
                    
                    // Signal TX thread that new frame is available
                    if (wake_handle_) {
                        wake_handle_->signal();
                    }
                }
            } else {
                error_count_.fetch_add(1);
            }
        } catch (const std::exception& e) {
            std::cerr << "[" << name_ << "] Capture error: " << e.what() << std::endl;
            error_count_.fetch_add(1);
        }
        
        // Frame rate control
        next_frame_time += target_period;
        auto sleep_until = std::max(next_frame_time, frame_start + std::chrono::microseconds(1000));
        std::this_thread::sleep_until(sleep_until);
    }
    
    std::cout << "[" << name_ << "] IR capture thread stopped. Frames: " 
              << frame_count_.load() << ", Errors: " << error_count_.load()
              << ", Discards: " << discard_count_.load() << std::endl;
}

bool IR_CaptureThread::capture_vospi_frame() {
    // Capture a frame
    for (int segment = 0; segment < vospi::SEGMENTS_PER_FRAME; segment++) {
        if (!capture_segment(segment)) {
            return false;
        }
        // Small delay between segments (if Lepton 3.5 support added later)
        if (segment < vospi::SEGMENTS_PER_FRAME - 1) {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }
    
    // Reconstruct the complete 160x120 frame
    reconstruct_frame();
    return true;
}

bool IR_CaptureThread::capture_segment(int segment_id) {
    uint8_t packet_buffer[vospi::PACKET_SIZE];
    int resets = 0;
    const int MAX_RESETS = 750; // Match GroupGets behavior
    static int debug_count = 0;
    
    // Simple for-loop like GroupGets, restart on mismatch
    while (running_.load()) {
        int packets_received = 0;
        
        // Read 60 packets for this segment
        for (int expected_line = 0; expected_line < vospi::PACKETS_PER_SEGMENT; expected_line++) {
            if (!read_vospi_packet(packet_buffer)) {
                return false;
            }
            
            // Check for discard packets (camera not ready)
            if (is_discard_packet(packet_buffer)) {
                discard_count_.fetch_add(1);
                usleep(1000);
                expected_line = -1; // Will become 0 in next iteration
                resets++;
                if (resets >= MAX_RESETS) {
                    // Too many resets, wait for camera to stabilize
                    std::this_thread::sleep_for(std::chrono::milliseconds(185));
                    resets = 0;
                }
                continue;
            }
            
            // Get line number from packet
            int line_number = get_packet_line_number(packet_buffer);
            
            // Debug first few iterations
            if (debug_count < 20) {
                std::cerr << "Line=" << line_number << " Expected=" << expected_line 
                          << " PktsRx=" << packets_received << " Resets=" << resets << std::endl;
                debug_count++;
            }
            
            // Validate line number is in range
            if (line_number < 0 || line_number >= vospi::LINES_PER_SEGMENT) {
                // Invalid line number, restart frame
                expected_line = -1; // Will become 0 in next iteration
                resets++;
                usleep(1000);
                if (resets >= MAX_RESETS) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(185));
                    resets = 0;
                }
                continue;
            }
            
            // For Lepton 2.5 (1 segment), line_number should match expected_line
            // Check if packet number matches expected
            if (line_number != expected_line) {
                // Mismatch - restart frame like GroupGets
                expected_line = -1; // Will become 0 in next iteration  
                resets++;
                usleep(1000);
                if (resets >= MAX_RESETS) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(185));
                    resets = 0;
                }
                continue;
            }
            
            // Valid packet! Copy payload to buffer
            int buffer_offset = packets_received * vospi::PAYLOAD_SIZE;
            std::memcpy(&segment_buffer_[buffer_offset], &packet_buffer[4], vospi::PAYLOAD_SIZE);
            
            packets_received++;
            resets = 0; // Reset counter on successful packet
        }
        
        // If we got all 60 packets, we're done!
        if (packets_received == vospi::PACKETS_PER_SEGMENT) {
            return true;
        }
        // Otherwise loop and try again
    }
    
    return false;
}

bool IR_CaptureThread::read_vospi_packet(uint8_t* packet_buffer) {
    struct spi_ioc_transfer transfer = {};
    transfer.tx_buf = 0; // No data to send
    transfer.rx_buf = reinterpret_cast<uintptr_t>(packet_buffer);
    transfer.len = vospi::PACKET_SIZE;
    transfer.speed_hz = config_.spi_speed;
    transfer.bits_per_word = 8;
    transfer.cs_change = 1;      // Deassert CS after transfer (critical for VoSPI sync)
    transfer.delay_usecs = 1;    // 1us delay after transfer
    
    int result = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &transfer);
    return result >= 0;
}

void IR_CaptureThread::reconstruct_frame() {
    // Convert segment buffer data to frame buffer
    for (int segment = 0; segment < vospi::SEGMENTS_PER_FRAME; segment++) {
        for (int line = 0; line < vospi::LINES_PER_SEGMENT; line++) {
            int frame_line = segment * vospi::LINES_PER_SEGMENT + line;
            int segment_offset = (segment * vospi::PACKETS_PER_SEGMENT * vospi::PAYLOAD_SIZE) + 
                                (line * vospi::PAYLOAD_SIZE);
            
            // Copy pixel data (80 pixels × 2 bytes = 160 bytes per line)
            for (int pixel = 0; pixel < vospi::PIXELS_PER_LINE; pixel++) {
                int frame_pixel_idx = frame_line * vospi::FRAME_WIDTH + pixel;
                int segment_byte_idx = segment_offset + (pixel * 2);
                
                // Convert big-endian to little-endian
                uint16_t pixel_value = (static_cast<uint16_t>(segment_buffer_[segment_byte_idx]) << 8) |
                                      static_cast<uint16_t>(segment_buffer_[segment_byte_idx + 1]);
                
                frame_buffer_[frame_pixel_idx] = pixel_value;
            }
        }
    }
}

std::shared_ptr<IRFrameHandle> IR_CaptureThread::create_frame_handle() {
    // Create OpenCV Mat from frame buffer
    cv::Mat thermal_frame(vospi::FRAME_HEIGHT, vospi::FRAME_WIDTH, CV_16UC1, frame_buffer_.data());
    
    // Create frame handle
    auto handle = std::make_shared<IRMatHandle>();
    handle->keep = std::make_shared<cv::Mat>(thermal_frame.clone()); // Clone to own the data
    
    // Setup frame metadata
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
    // Sync/first packet has line number 0 (packet[1] = 0x00)
    // For Lepton 2.5, packet[0] should be 0x00 for segment 0
    return (packet[0] == 0x00) && (packet[1] == 0x00);
}

bool IR_CaptureThread::is_discard_packet(const uint8_t* packet) {
    // Discard packets have lower 4 bits of packet[0] == 0x0F
    // Reference: groupgets/LeptonModule raspberrypi_capture.c line 132
    return ((packet[0] & 0x0F) == 0x0F);
}

int IR_CaptureThread::get_packet_line_number(const uint8_t* packet) {
    // Line number is in bytes 0-1 (ID field)
    // VoSPI packet ID field format (16-bit):
    //   Bits [7:0]: Packet/line number (0-59 for Lepton 2.5, one segment)
    //   Bits [11:8]: Segment number (always 0 for Lepton 2.5)
    //   Bits [15:12]: Reserved/flags
    // 
    // The packet bytes come over SPI in big-endian order:
    //   packet[0] = ID[15:8] (upper byte: segment + reserved bits)
    //   packet[1] = ID[7:0]  (lower byte: line number)
    //
    // For Lepton 2.5 (1 segment), we only care about packet[1] for line number
    return static_cast<int>(packet[1]); // Line number is in lower byte (0-59)
}

uint64_t IR_CaptureThread::get_timestamp_ns() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

} // namespace flir

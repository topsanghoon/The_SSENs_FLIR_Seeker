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
    const IRCaptureConfig& config)
    : name_(std::move(name))
    , output_mailbox_(output_mailbox)
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
    uint8_t spi_mode = SPI_MODE_3;
    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &spi_mode) < 0) {
        std::cerr << "[" << name_ << "] Failed to set SPI mode" << std::endl;
        close(spi_fd_);
        return false;
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
    
    std::cout << "[" << name_ << "] SPI initialized: " << config_.spi_device 
              << " @ " << spi_speed << " Hz" << std::endl;
    
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
    // Capture all 4 segments for a complete frame
    for (int segment = 0; segment < vospi::SEGMENTS_PER_FRAME; segment++) {
        if (!capture_segment(segment)) {
            return false;
        }
    }
    
    // Reconstruct the complete 160x120 frame
    reconstruct_frame();
    return true;
}

bool IR_CaptureThread::capture_segment(int segment_id) {
    uint8_t packet_buffer[vospi::PACKET_SIZE];
    int packets_received = 0;
    int sync_attempts = 0;
    const int MAX_SYNC_ATTEMPTS = 750; // Max attempts to find sync
    
    while (packets_received < vospi::PACKETS_PER_SEGMENT && running_.load()) {
        if (!read_vospi_packet(packet_buffer)) {
            return false;
        }
        
        // Check for discard packets (camera not ready)
        if (is_discard_packet(packet_buffer)) {
            discard_count_.fetch_add(1);
            packets_received = 0; // Reset segment capture
            sync_attempts++;
            if (sync_attempts > MAX_SYNC_ATTEMPTS) {
                return false;
            }
            continue;
        }
        
        // Get line number from packet
        int line_number = get_packet_line_number(packet_buffer);
        
        // Validate line number for current segment
        int expected_line = packets_received + (segment_id * vospi::LINES_PER_SEGMENT);
        if (line_number != expected_line) {
            // Out of sync, restart segment capture
            packets_received = 0;
            sync_attempts++;
            if (sync_attempts > MAX_SYNC_ATTEMPTS) {
                return false;
            }
            continue;
        }
        
        // Copy packet payload to segment buffer
        int buffer_offset = packets_received * vospi::PAYLOAD_SIZE;
        std::memcpy(&segment_buffer_[buffer_offset], &packet_buffer[4], vospi::PAYLOAD_SIZE);
        
        packets_received++;
        sync_attempts = 0; // Reset sync attempts on successful packet
    }
    
    return packets_received == vospi::PACKETS_PER_SEGMENT;
}

bool IR_CaptureThread::read_vospi_packet(uint8_t* packet_buffer) {
    struct spi_ioc_transfer transfer = {};
    transfer.tx_buf = 0; // No data to send
    transfer.rx_buf = reinterpret_cast<uintptr_t>(packet_buffer);
    transfer.len = vospi::PACKET_SIZE;
    transfer.speed_hz = config_.spi_speed;
    transfer.bits_per_word = 8;
    
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
    // Sync packets have line number 0x0000
    return (packet[0] == 0x00) && (packet[1] == 0x00);
}

bool IR_CaptureThread::is_discard_packet(const uint8_t* packet) {
    // Discard packets have line number 0x0F00
    return (packet[0] == 0x0F) && (packet[1] == 0x00);
}

int IR_CaptureThread::get_packet_line_number(const uint8_t* packet) {
    // Line number is in bytes 0-1 (big-endian format)
    return (static_cast<int>(packet[0]) << 8) | static_cast<int>(packet[1]);
}

uint64_t IR_CaptureThread::get_timestamp_ns() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

} // namespace flir

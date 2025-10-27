#include "threads_includes/EO_CaptureThread.hpp"
#include <iostream>
#include <chrono>

namespace flir {

EO_CaptureThread::EO_CaptureThread(
    std::string name,
    SpscMailbox<std::shared_ptr<EOFrameHandle>>& output_mb,
    const EOCaptureConfig& cfg)
    : name_(std::move(name))
    , output_mb_(output_mb)
    , cfg_(cfg)
{
}

EO_CaptureThread::~EO_CaptureThread() {
    stop();
    join();
    cleanup_camera();
}

void EO_CaptureThread::start() {
    if (th_.joinable()) return;
    
    if (!initialize_camera()) {
        throw std::runtime_error("EO_CaptureThread failed to initialize camera");
    }
    
    running_.store(true);
    th_ = std::thread(&EO_CaptureThread::run, this);
}

void EO_CaptureThread::stop() {
    running_.store(false);
}

void EO_CaptureThread::join() {
    if (th_.joinable()) {
        th_.join();
    }
}

bool EO_CaptureThread::initialize_camera() {
    // Open camera with V4L2 backend
    cap_.open(cfg_.device_id, cv::CAP_V4L2);
    
    if (!cap_.isOpened()) {
        log_debug("Failed to open camera device " + std::to_string(cfg_.device_id));
        return false;
    }
    
    // Set capture properties
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, cfg_.width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cfg_.height);
    cap_.set(cv::CAP_PROP_FPS, cfg_.fps);
    cap_.set(cv::CAP_PROP_FOURCC, cfg_.fourcc);
    
    // Verify settings
    int actual_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    int actual_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actual_fps = cap_.get(cv::CAP_PROP_FPS);
    
    log_debug("Camera initialized: " + std::to_string(actual_width) + "x" + std::to_string(actual_height) 
              + " @ " + std::to_string(actual_fps) + " fps");
    
    return true;
}

void EO_CaptureThread::cleanup_camera() {
    if (cap_.isOpened()) {
        cap_.release();
    }
}

void EO_CaptureThread::run() {
    log_debug("EO capture thread started");
    
    auto target_period = std::chrono::microseconds(1000000 / cfg_.fps);
    auto next_frame_time = std::chrono::steady_clock::now();
    
    cv::Mat yuv_frame;
    
    while (running_.load()) {
        auto frame_start = std::chrono::steady_clock::now();
        
        try {
            // Capture frame from camera
            if (!cap_.read(yuv_frame)) {
                log_debug("Failed to capture frame");
                error_count_.fetch_add(1);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            
            if (yuv_frame.empty()) {
                error_count_.fetch_add(1);
                continue;
            }
            
            // Create frame handle and push to mailbox
            auto frame_handle = create_frame_handle(yuv_frame);
            if (frame_handle) {
                output_mb_.push(frame_handle);
                frame_count_.fetch_add(1);
            }
            
        } catch (const std::exception& e) {
            log_debug("Capture error: " + std::string(e.what()));
            error_count_.fetch_add(1);
        }
        
        // Frame rate control
        next_frame_time += target_period;
        auto sleep_until = std::max(next_frame_time, frame_start + std::chrono::microseconds(1000));
        std::this_thread::sleep_until(sleep_until);
    }
    
    log_debug("EO capture thread stopped. Frames: " + std::to_string(frame_count_.load()) + 
              ", Errors: " + std::to_string(error_count_.load()));
}

std::shared_ptr<EOFrameHandle> EO_CaptureThread::create_frame_handle(const cv::Mat& yuv_frame) {
    // Convert YUV to BGR
    cv::Mat bgr_frame;
    
    // Handle different YUV formats
    if (yuv_frame.channels() == 2) {
        // YUV422 format (YUYV)
        cv::cvtColor(yuv_frame, bgr_frame, cv::COLOR_YUV2BGR_YUYV);
    } else if (yuv_frame.channels() == 3) {
        // Already in color format, assume BGR
        bgr_frame = yuv_frame.clone();
    } else {
        // Grayscale, convert to BGR
        cv::cvtColor(yuv_frame, bgr_frame, cv::COLOR_GRAY2BGR);
    }
    
    // Create frame handle
    auto handle = std::make_shared<EOMatHandle>();
    handle->keep = std::make_shared<cv::Mat>(bgr_frame.clone()); // Clone to own the data
    
    // Setup frame metadata
    auto& owned = handle->owned;
    owned.data = handle->keep->ptr<uint8_t>(0);
    owned.width = handle->keep->cols;
    owned.height = handle->keep->rows;
    owned.step = static_cast<int>(handle->keep->step);
    
    handle->seq = sequence_.fetch_add(1);
    handle->ts = get_timestamp_ns();
    
    return handle;
}

uint64_t EO_CaptureThread::get_timestamp_ns() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

void EO_CaptureThread::log_debug(const std::string& msg) {
    std::cout << "[" << name_ << "] " << msg << std::endl;
}

} // namespace flir

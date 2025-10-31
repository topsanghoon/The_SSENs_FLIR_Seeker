#include "threads_includes/EO_CaptureThread.hpp"
#include <iostream>
#include <chrono>

namespace flir {

EO_CaptureThread::EO_CaptureThread(
    std::string name,
    SpscMailbox<std::shared_ptr<EOFrameHandle>>& output_mailbox,
    std::unique_ptr<WakeHandle> wake_handle,
    const EOCaptureConfig& config)
    : name_(std::move(name))
    , output_mailbox_(output_mailbox)
    , wake_handle_(std::move(wake_handle))
    , config_(config)
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
    cap_.open(config_.device_id, cv::CAP_V4L2);
    
    if (!cap_.isOpened()) {
        std::cerr << "[" << name_ << "] Failed to open camera device " << config_.device_id << std::endl;
        return false;
    }
    
    // Set capture properties
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, config_.width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.height);
    cap_.set(cv::CAP_PROP_FPS, config_.fps);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    //
    printf("%d", cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    printf("%d", cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("%d", cap_.get(cv::CAP_PROP_FPS));
    printf("%d", cap_.get(cv::CAP_PROP_FOURCC));
    
    // Verify settings
    int actual_width = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    int actual_height = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
    double actual_fps = cap_.get(cv::CAP_PROP_FPS);
    
    std::cout << "[" << name_ << "] Camera initialized: " << actual_width << "x" << actual_height 
              << " @ " << actual_fps << " fps" << std::endl;
    
    return true;
}

void EO_CaptureThread::cleanup_camera() {
    if (cap_.isOpened()) {
        cap_.release();
    }
}

void EO_CaptureThread::run() {
    std::cout << "[" << name_ << "] EO capture thread started" << std::endl;
    
    auto target_period = std::chrono::microseconds(1000000 / config_.fps);
    auto next_frame_time = std::chrono::steady_clock::now();
    
    cv::Mat captured_frame;
    
    while (running_.load()) {
        auto frame_start = std::chrono::steady_clock::now();
        
        try {
            // Capture frame from camera
            if (!cap_.read(captured_frame)) {
                std::cerr << "[" << name_ << "] Failed to capture frame" << std::endl;
                error_count_.fetch_add(1);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            
            if (captured_frame.empty()) {
                error_count_.fetch_add(1);
                continue;
            }
            
            // Debug: Check frame format
            static int debug_count = 0;
            if (debug_count < 5) {
                std::cout << "[" << name_ << "] Frame " << debug_count << ": " 
                          << captured_frame.cols << "x" << captured_frame.rows 
                          << " type=" << captured_frame.type() 
                          << " channels=" << captured_frame.channels() 
                          << " depth=" << captured_frame.depth() << std::endl;
                debug_count++;
            }
            
            // Create frame handle and push to mailbox
            auto frame_handle = create_frame_handle(captured_frame);
            if (frame_handle) {
                output_mailbox_.push(frame_handle);
                frame_count_.fetch_add(1);
                
                // Signal TX thread that new frame is available
                if (wake_handle_) {
                    wake_handle_->signal();
                }
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
    
    std::cout << "[" << name_ << "] EO capture thread stopped. Frames: " 
              << frame_count_.load() << ", Errors: " << error_count_.load() << std::endl;
}

std::shared_ptr<EOFrameHandle> EO_CaptureThread::create_frame_handle(const cv::Mat& captured_frame) {
    // Create frame handle directly from captured frame (MJPEG already decompressed to BGR by OpenCV)
    auto handle = std::make_shared<EOMatHandle>();
    handle->keep = std::make_shared<cv::Mat>(captured_frame.clone()); // Direct clone - no format conversion needed
    
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

} // namespace flir
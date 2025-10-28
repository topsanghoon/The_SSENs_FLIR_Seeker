#pragma once
#include <atomic>
#include <thread>
#include <string>
#include <memory>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "components/includes/EO_Frame.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/wake.hpp"

namespace flir {

struct EOCaptureConfig {
    int device_id = 0;           // V4L2 device ID (usually /dev/video0)
    int width = 320;             // Capture width
    int height = 240;            // Capture height  
    int fps = 15;                // Target frame rate
    int fourcc = cv::VideoWriter::fourcc('M','J','P','G');  // MJPEG format for MP4/MOV compatible cameras
};

// MatHandle implementation for EO frames
struct EOMatHandle : EOFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    FrameBGR8 owned{};
    EOMatHandle() { p = &owned; }
    
    ~EOMatHandle() override = default;
    
    void retain() override {
        // shared_ptr handles reference counting
    }
    
    void release() override {
        // shared_ptr handles reference counting
    }
};

class EO_CaptureThread {
public:
    EO_CaptureThread(
        std::string name,
        SpscMailbox<std::shared_ptr<EOFrameHandle>>& output_mailbox,
        std::unique_ptr<WakeHandle> wake_handle,
        const EOCaptureConfig& config = EOCaptureConfig{}
    );
    
    ~EO_CaptureThread();
    
    void start();
    void stop();
    void join();
    
    // Statistics
    uint64_t get_frame_count() const { return frame_count_.load(); }
    uint64_t get_error_count() const { return error_count_.load(); }
    
private:
    std::string name_;
    SpscMailbox<std::shared_ptr<EOFrameHandle>>& output_mailbox_;
    std::unique_ptr<WakeHandle> wake_handle_;
    EOCaptureConfig config_;
    
    // Thread management
    std::thread th_;
    std::atomic<bool> running_{false};
    
    // OpenCV VideoCapture
    cv::VideoCapture cap_;
    
    // Statistics
    std::atomic<uint64_t> frame_count_{0};
    std::atomic<uint64_t> error_count_{0};
    std::atomic<uint32_t> sequence_{0};
    
    // Internal methods
    void run();
    bool initialize_camera();
    void cleanup_camera();
    std::shared_ptr<EOFrameHandle> create_frame_handle(const cv::Mat& yuv_frame);
    uint64_t get_timestamp_ns();
};

} // namespace flir

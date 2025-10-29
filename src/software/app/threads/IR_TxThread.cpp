#include "threads_includes/IR_TxThread.hpp"
#include "ipc/wake_condvar.hpp" 
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <iostream>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

namespace flir {

IR_TxThread::IR_TxThread(std::string name, SpscMailbox<std::shared_ptr<IRFrameHandle>>& mb, const GstConfig& gst_config)
    : name_(std::move(name)),
      mb_(mb),
      gst_config_(gst_config)
{}

// Default constructor using default GstConfig values
IR_TxThread::IR_TxThread(std::string name, SpscMailbox<std::shared_ptr<IRFrameHandle>>& mb)
    : name_(std::move(name)),
      mb_(mb),
      gst_config_(GstConfig{}) // Use default values
{}

// 소멸자: GStreamer 리소스 정리
IR_TxThread::~IR_TxThread() {
    stop();
    join();
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }
}

// GStreamer 파이프라인 초기화 
bool IR_TxThread::initialize_gstreamer() {
    static bool gst_initialized = false;
    if (!gst_initialized) {
        gst_init(nullptr, nullptr);
        gst_initialized = true;
    }

    std::stringstream ss;
    ss << "appsrc name=ir_appsrc is-live=true do-timestamp=true block=true format=TIME "
       << "caps=video/x-raw,format=GRAY8,width=" << gst_config_.width
       << ",height=" << gst_config_.height
       << ",framerate=" << gst_config_.fps << "/1 ! "
       << "videoconvert ! video/x-raw,format=I420 ! "
       << "jpegenc ! rtpjpegpay pt=26 mtu=1200 ! "
       << "udpsink host=" << gst_config_.pc_ip
       << " port=" << gst_config_.port
       << " sync=false async=false";
       
    std::string pipeline_str = ss.str();
    
    std::cout << "[" << name_ << "] GStreamer pipeline: " << pipeline_str << std::endl;

    GError* error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error || !pipeline_) {
        std::cerr << "[" << name_ << "] ERROR: Failed to create pipeline";
        if (error) {
            std::cerr << ": " << error->message;
            g_error_free(error);
        }
        std::cerr << std::endl;
        return false;
    }

    appsrc_ = (GstAppSrc*)gst_bin_get_by_name(GST_BIN(pipeline_), "ir_appsrc");
    if (!appsrc_) {
        std::cerr << "[" << name_ << "] ERROR: Failed to get appsrc element" << std::endl;
        return false;
    }
    
    g_object_set(G_OBJECT(appsrc_), "block", TRUE, nullptr);
    
    auto state_ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (state_ret == GST_STATE_CHANGE_FAILURE) {
        std::cerr << "[" << name_ << "] ERROR: Failed to set pipeline to PLAYING state" << std::endl;
        return false;
    }
    
    std::cout << "[" << name_ << "] GStreamer pipeline started successfully" << std::endl;
    return true;
}

// 스레드 시작
void IR_TxThread::start() {
    if (th_.joinable()) return;
    if (!initialize_gstreamer()) {
        throw std::runtime_error("IR_TxThread failed to initialize GStreamer.");
    }
    running_.store(true);
    th_ = std::thread(&IR_TxThread::run, this);
}

void IR_TxThread::stop() { running_.store(false); cv_.notify_one(); }
void IR_TxThread::join() { if (th_.joinable()) th_.join(); }

std::unique_ptr<WakeHandle> IR_TxThread::create_wake_handle() {
    return std::make_unique<WakeHandleCondVar>(cv_);
}

void IR_TxThread::wait_for_frame() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return !running_.load() || mb_.has_new(frame_seq_seen_); });
}

void IR_TxThread::push_frame_to_gst(const std::shared_ptr<IRFrameHandle>& handle) {
    if (!handle || !handle->p || !handle->p->data) return;

    const int num_pixels = gst_config_.width * gst_config_.height;
    const uint16_t* src16 = reinterpret_cast<const uint16_t*>(handle->p->data);
    
    // Find min/max for normalization (like working lepton code)
    uint16_t minVal = 65535, maxVal = 0;
    for (int i = 0; i < num_pixels; ++i) {
        if (src16[i] < minVal) minVal = src16[i];
        if (src16[i] > maxVal) maxVal = src16[i];
    }
    
    // Prevent division by zero
    if (maxVal <= minVal) maxVal = minVal + 1;
    
    // Allocate GRAY8 buffer (will be freed by GStreamer callback)
    uint8_t* gray8_data = new uint8_t[num_pixels];
    
    // Convert GRAY16_LE to GRAY8 with normalization
    double scale = 255.0 / (maxVal - minVal);
    for (int i = 0; i < num_pixels; ++i) {
        int val = static_cast<int>((src16[i] - minVal) * scale);
        gray8_data[i] = static_cast<uint8_t>(std::min(255, std::max(0, val)));
    }
    
    // Package both buffers for cleanup
    struct BufferCleanup {
        uint8_t* gray8_buffer;
        std::shared_ptr<IRFrameHandle> handle;
    };
    auto* cleanup_data = new BufferCleanup{gray8_data, handle};

    GstBuffer* buffer = gst_buffer_new_wrapped_full(
        (GstMemoryFlags)0,
        (gpointer)gray8_data,
        num_pixels,  // GRAY8 is 1 byte per pixel
        0,
        num_pixels,
        cleanup_data,
        [](gpointer user_data) {
            auto* cleanup = static_cast<BufferCleanup*>(user_data);
            delete[] cleanup->gray8_buffer;  // Free GRAY8 buffer
            delete cleanup;                   // Free cleanup struct
        }
    );

    GST_BUFFER_PTS(buffer) = handle->ts;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, gst_config_.fps);

    GstFlowReturn ret = gst_app_src_push_buffer((GstAppSrc*)appsrc_, buffer);

    if (ret != GST_FLOW_OK) {
        std::cerr << "[" << name_ << "] WARNING: gst_app_src_push_buffer failed with code: " << ret << std::endl;
    }
}

// 스레드 메인 루프
void IR_TxThread::run() {
    std::cout << "[" << name_ << "] IR TX thread started, streaming to " 
              << gst_config_.pc_ip << ":" << gst_config_.port << std::endl;
    
    uint64_t frames_sent = 0;
    
    while (running_.load()) {
        wait_for_frame();
        if (!running_.load()) break;

        if (auto handle_opt = mb_.exchange(nullptr)) {
            push_frame_to_gst(*handle_opt);
            frames_sent++;
            
            // Debug: Print every 30 frames
            if (frames_sent % 30 == 0) {
                std::cout << "[" << name_ << "] Sent " << frames_sent << " frames" << std::endl;
            }
        }
        
        // Always update frame_seq_seen_ like EO_TxThread
        frame_seq_seen_ = mb_.latest_seq();
    }
    
    std::cout << "[" << name_ << "] IR TX thread stopped, total frames sent: " << frames_sent << std::endl;
}

} // namespace flir
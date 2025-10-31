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
    
    // Send EOS before joining thread
    if (pipeline_ && appsrc_) {
        gst_app_src_end_of_stream((GstAppSrc*)appsrc_);
    }
    
    join();
    
    if (pipeline_) {
        // Wait for EOS to propagate
        GstBus* bus = gst_element_get_bus(pipeline_);
        if (bus) {
            gst_bus_timed_pop_filtered(bus, 500 * GST_MSECOND, 
                                       (GstMessageType)(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
            gst_object_unref(bus);
        }
        
        // Proper shutdown sequence
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        
        // Wait for state change to complete
        GstStateChangeReturn ret = gst_element_get_state(pipeline_, nullptr, nullptr, GST_SECOND);
        if (ret == GST_STATE_CHANGE_FAILURE) {
            std::cerr << "[" << name_ << "] WARNING: Failed to set pipeline to NULL state" << std::endl;
        }
        
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        appsrc_ = nullptr;
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
    
    // Find current frame min/max (per-frame normalization)
    uint16_t minVal = 65535, maxVal = 0;
    for (int i = 0; i < num_pixels; ++i) {
        if (src16[i] < minVal) minVal = src16[i];
        if (src16[i] > maxVal) maxVal = src16[i];
    }
    
    // Add margin for better contrast
    int range = maxVal - minVal;
    double margin = range * 0.1;
    if (margin < 100) margin = 100;
    
    minVal = std::max(0, static_cast<int>(minVal) - static_cast<int>(margin));
    maxVal = std::min(16383, static_cast<int>(maxVal) + static_cast<int>(margin));
    
    if (maxVal <= minVal) maxVal = minVal + 1;
    
    // Create GStreamer buffer and map for writing
    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, num_pixels, nullptr);
    if (!buffer) {
        std::cerr << "[" << name_ << "] ERROR: Failed to allocate GStreamer buffer" << std::endl;
        return;
    }
    
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        std::cerr << "[" << name_ << "] ERROR: Failed to map GStreamer buffer" << std::endl;
        gst_buffer_unref(buffer);
        return;
    }
    
    // Convert GRAY16_LE to GRAY8 directly into GStreamer buffer
    uint8_t* gray8_data = map.data;
    double scale = 255.0 / (maxVal - minVal);
    for (int i = 0; i < num_pixels; ++i) {
        int val = static_cast<int>((src16[i] - minVal) * scale);
        gray8_data[i] = static_cast<uint8_t>(std::min(255, std::max(0, val)));
    }
    
    gst_buffer_unmap(buffer, &map);

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
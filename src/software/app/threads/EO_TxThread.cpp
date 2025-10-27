#include "threads_includes/EO_TxThread.hpp"
#include "ipc/wake_condvar.hpp" 
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <iostream>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

namespace flir {

EO_TxThread::EO_TxThread(std::string name, SpscMailbox<std::shared_ptr<EOFrameHandle>>& mb, const GstConfig& cfg)
    : name_(std::move(name)),
      mb_(mb),
      gst_config_(cfg)
{}

// Default constructor using default GstConfig values
EO_TxThread::EO_TxThread(std::string name, SpscMailbox<std::shared_ptr<EOFrameHandle>>& mb)
    : name_(std::move(name)),
      mb_(mb),
      gst_config_(GstConfig{}) // Use default values
{}

// 소멸자: GStreamer 리소스 정리
EO_TxThread::~EO_TxThread() {
    stop();
    join();
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }
}

// GStreamer 파이프라인 초기화 - YUV422를 I420로 변환하여 jpegenc 스트리밍
bool EO_TxThread::initialize_gstreamer() {
    static bool gst_initialized = false;
    if (!gst_initialized) {
        gst_init(nullptr, nullptr);
        gst_initialized = true;
    }

    std::stringstream ss;
    ss << "appsrc name=eo_appsrc format=GST_FORMAT_TIME is-live=true ! "
       << "video/x-raw,format=BGR,width=" << gst_config_.width
       << ",height=" << gst_config_.height
       << ",framerate=" << gst_config_.fps << "/1 ! "
       << "videoconvert ! "
       << "jpegenc ! "
       << "udpsink host=" << gst_config_.pc_ip
       << " port=" << gst_config_.port;
       
    std::string pipeline_str = ss.str();

    GError* error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error || !pipeline_) {
        if(error) g_error_free(error);
        return false;
    }

    appsrc_ = (GstAppSrc*)gst_bin_get_by_name(GST_BIN(pipeline_), "eo_appsrc");
    if (!appsrc_) {
        return false;
    }
    
    g_object_set(G_OBJECT(appsrc_), "block", TRUE, nullptr);
    
    return gst_element_set_state(pipeline_, GST_STATE_PLAYING) != GST_STATE_CHANGE_FAILURE;
}

// 스레드 시작
void EO_TxThread::start() {
    if (th_.joinable()) return;
    if (!initialize_gstreamer()) {
        throw std::runtime_error("EO_TxThread failed to initialize GStreamer.");
    }
    running_.store(true);
    th_ = std::thread(&EO_TxThread::run, this);
}

void EO_TxThread::stop() { 
    running_.store(false); 
    cv_.notify_one(); 
}
void EO_TxThread::join() { 
    if (th_.joinable()) th_.join(); 
}

std::unique_ptr<WakeHandle> EO_TxThread::create_wake_handle() {
    return std::make_unique<WakeHandleCondVar>(cv_);
}

void EO_TxThread::wait_for_frame() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return !running_.load() || mb_.has_new(frame_seq_seen_); });
}

void EO_TxThread::push_frame_to_gst(const std::shared_ptr<EOFrameHandle>& handle) {
    if (!handle || !handle->p || !handle->p->data) return;

    // Use the actual frame step for buffer size calculation
    guint buffer_size = handle->p->step * handle->p->height;

    // GStreamer 콜백이 비동기적으로 호출되므로, shared_ptr의 생명주기를 콜백까지 연장해야 함.
    // 힙에 shared_ptr의 복사본을 만들어 포인터를 콜백의 user_data로 전달.
    auto* handle_copy = new std::shared_ptr<EOFrameHandle>(handle);

    GstBuffer* buffer = gst_buffer_new_wrapped_full(
        GST_MEMORY_FLAG_READONLY,
        (gpointer)handle->p->data,
        buffer_size,
        0,
        buffer_size,
        handle_copy, // 힙에 할당된 shared_ptr 포인터를 전달
        [](gpointer user_data) {
            // user_data를 원래 타입(shared_ptr 포인터)으로 캐스팅
            auto* h_ptr = static_cast<std::shared_ptr<EOFrameHandle>*>(user_data);
            // GStreamer가 버퍼 사용을 마쳤으므로 힙에 할당했던 shared_ptr를 delete.
            // 이 과정에서 shared_ptr의 소멸자가 호출되어 참조 카운트가 안전하게 감소.
            delete h_ptr;
        }
    );

    GST_BUFFER_PTS(buffer) = handle->ts;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, gst_config_.fps);

    GstFlowReturn ret = gst_app_src_push_buffer((GstAppSrc*)appsrc_, buffer);

    if (ret != GST_FLOW_OK) {
        // log_.error("gst_app_src_push_buffer failed with code: " + std::to_string(ret));
    }
}

// 스레드 메인 루프
void EO_TxThread::run() {
    log_debug("EO Tx thread started");
    while (running_.load()) {
        wait_for_frame();
        if (!running_.load()) break;

        if (auto handle_opt = mb_.exchange(nullptr)) {
            push_frame_to_gst(*handle_opt);
        }
        
        frame_seq_seen_ = mb_.latest_seq();
    }
    log_debug("EO Tx thread stopped");
}

void EO_TxThread::log_debug(const std::string& msg) {
    std::cout << "[" << name_ << "] " << msg << std::endl;
}

} // namespace flir
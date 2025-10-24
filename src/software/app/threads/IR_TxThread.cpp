// IR_TxThread.cpp
#include "threads_includes/IR_TxThread.hpp"
#include "ipc/wake_condvar.hpp"

#include <chrono>
#include <sstream>
#include <stdexcept>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include "components/includes/common_log.hpp"   // ★ 컴파일타임 로거

namespace flir {

static constexpr const char* TAG = "IR_Tx";

IR_TxThread::IR_TxThread(std::string name,
                         SpscMailbox<std::shared_ptr<IRFrameHandle>>& mb,
                         const GstConfig& gst_config)
    : name_(std::move(name)), mb_(mb), gst_config_(gst_config) {}

IR_TxThread::IR_TxThread(std::string name,
                         SpscMailbox<std::shared_ptr<IRFrameHandle>>& mb)
    : name_(std::move(name)), mb_(mb), gst_config_(GstConfig{}) {}

IR_TxThread::~IR_TxThread() {
    stop();
    join();
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        appsrc_ = nullptr;
    }
}

bool IR_TxThread::initialize_gstreamer() {
    static bool gst_initialized = false;
    if (!gst_initialized) {
        gst_init(nullptr, nullptr);
        gst_initialized = true;
        LOGI(TAG, "gst_init()");
    }

    std::stringstream ss;
    ss << "appsrc name=ir_appsrc format=GST_FORMAT_TIME is-live=true ! "
       << "videoparse width=" << gst_config_.width
       << " height=" << gst_config_.height
       << " framerate=" << gst_config_.fps << "/1"
       << " format=gray16-le ! "
       << "udpsink host=" << gst_config_.pc_ip
       << " port=" << gst_config_.port;

    const std::string pipeline_str = ss.str();
    LOGI(TAG, "pipeline: %s", pipeline_str.c_str());

    GError* error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error || !pipeline_) {
        LOGE(TAG, "gst_parse_launch failed: %s", error ? error->message : "nullptr");
        if (error) g_error_free(error);
        return false;
    }

    appsrc_ = (GstAppSrc*)gst_bin_get_by_name(GST_BIN(pipeline_), "ir_appsrc");
    if (!appsrc_) {
        LOGE(TAG, "get_by_name(ir_appsrc) failed");
        return false;
    }

    g_object_set(G_OBJECT(appsrc_), "block", TRUE, nullptr);

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        LOGE(TAG, "set_state(PLAYING) failed");
        return false;
    }

    LOGI(TAG, "GStreamer IR pipeline started");
    return true;
}

void IR_TxThread::start() {
    if (th_.joinable()) return;
    if (!initialize_gstreamer()) {
        throw std::runtime_error("IR_TxThread failed to initialize GStreamer.");
    }
    running_.store(true);
    th_ = std::thread(&IR_TxThread::run, this);
    LOGI(TAG, "thread started");
}

void IR_TxThread::stop() {
    running_.store(false);
    cv_.notify_one();
    LOGI(TAG, "stop requested");
}

void IR_TxThread::join() {
    if (th_.joinable()) {
        th_.join();
        LOGI(TAG, "thread joined");
    }
}

std::unique_ptr<WakeHandle> IR_TxThread::create_wake_handle() {
    return std::make_unique<WakeHandleCondVar>(cv_);
}

void IR_TxThread::wait_for_frame() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return !running_.load() || mb_.has_new(frame_seq_seen_); });
}

void IR_TxThread::push_frame_to_gst(const std::shared_ptr<IRFrameHandle>& handle) {
    if (!handle || !handle->p || !handle->p->data) return;

    const guint buffer_size =
        static_cast<guint>(gst_config_.width * gst_config_.height * sizeof(uint16_t));

    // GStreamer가 버퍼를 다 쓸 때까지 안전하게 보관
    auto* handle_copy = new std::shared_ptr<IRFrameHandle>(handle);

    GstBuffer* buffer = gst_buffer_new_wrapped_full(
        GST_MEMORY_FLAG_READONLY,
        (gpointer)handle->p->data,
        buffer_size,
        0,
        buffer_size,
        handle_copy,
        [](gpointer user_data) {
            auto* h_ptr = static_cast<std::shared_ptr<IRFrameHandle>*>(user_data);
            delete h_ptr;
        });

    // 타임스탬프/길이
    GST_BUFFER_PTS(buffer)      = handle->ts; // 상위에서 ns 단위로 관리 중이라 가정
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, gst_config_.fps);

    const GstFlowReturn ret = gst_app_src_push_buffer((GstAppSrc*)appsrc_, buffer);
    if (ret != GST_FLOW_OK) {
        LOGE(TAG, "gst_app_src_push_buffer failed: %d", (int)ret);
    } else {
        LOGD(TAG, "pushed frame seq=%u bytes=%u", handle->seq, buffer_size);
    }
}

void IR_TxThread::run() {
    while (running_.load()) {
        wait_for_frame();
        if (!running_.load()) break;

        if (auto handle_opt = mb_.exchange(nullptr)) {
            push_frame_to_gst(*handle_opt);
        }
        frame_seq_seen_ = mb_.latest_seq();
    }
    LOGI(TAG, "run() exit");
}

} // namespace flir

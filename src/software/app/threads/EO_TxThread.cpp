// EO_TxThread.cpp (CSV unified, no per-IPC CSV logs)
#include "threads_includes/EO_TxThread.hpp"

#include <chrono>
#include <sstream>
#include <stdexcept>
#include "util/telemetry.hpp"
#include "util/common_log.hpp"   // LOGI/LOGD/LOGE
#include "util/time_util.hpp"    // ScopedTimerMs
#include "util/csv_sink.hpp"     // CSV_LOG_SIMPLE

namespace flir {

static constexpr const char* TAG = "EO_Tx";

EO_TxThread::EO_TxThread(std::string name,
                         AppConfigPtr cfg,
                         SpscMailbox<std::shared_ptr<EOFrameHandle>>& mb,
                         WakeHandle& /*unused*/)
    : name_(std::move(name))
    , cfg_(std::move(cfg))
    , mb_(mb) {}

EO_TxThread::~EO_TxThread() {
    stop();
    join();
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        appsrc_   = nullptr;
    }
}

bool EO_TxThread::initialize_gstreamer() {
    static bool gst_initialized = false;
    if (!gst_initialized) {
        gst_init(nullptr, nullptr);
        gst_initialized = true;
        LOGI(TAG, "gst_init()");
    }

    // appsrc(BGR) → videoconvert → jpegenc → udpsink
    std::stringstream ss;
    ss << "appsrc name=eo_appsrc is-live=true do-timestamp=true block=false "
       << "! video/x-raw,format=BGR,width="  << cfg_->eo_tx.frame.width
       << ",height="                         << cfg_->eo_tx.frame.height
       << ",framerate="                      << cfg_->eo_tx.fps << "/1 "
       << "! queue max-size-buffers=8 max-size-time=0 leaky=downstream "
       << "! videoconvert "
       << "! jpegenc quality="               << cfg_->eo_tx.jpeg_quality << " "
       << "! udpsink host="                  << cfg_->eo_tx.dst.ip
       << " port="                           << cfg_->eo_tx.dst.port;

    const std::string pipeline_str = ss.str();
    LOGI(TAG, "EO pipeline: %s", pipeline_str.c_str());

    GError* error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error || !pipeline_) {
        LOGE(TAG, "gst_parse_launch failed: %s", error ? error->message : "nullptr");
        if (error) g_error_free(error);
        return false;
    }

    appsrc_ = (GstAppSrc*)gst_bin_get_by_name(GST_BIN(pipeline_), "eo_appsrc");
    if (!appsrc_) {
        LOGE(TAG, "get_by_name(eo_appsrc) failed");
        return false;
    }

    // 다운스트림이 막히면 block 하도록(원래 코드와 동일)
    g_object_set(G_OBJECT(appsrc_), "block", TRUE, nullptr);

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        LOGE(TAG, "set_state(PLAYING) failed");
        return false;
    }

    LOGI(TAG, "EO_TxThread init done (dst=%s:%u %dx%d@%dfps q=%d)",
         cfg_->eo_tx.dst.ip.c_str(), (unsigned)cfg_->eo_tx.dst.port,
         cfg_->eo_tx.frame.width, cfg_->eo_tx.frame.height, cfg_->eo_tx.fps,
         cfg_->eo_tx.jpeg_quality);
    return true;
}

void EO_TxThread::start() {
    if (th_.joinable()) return;
    if (!initialize_gstreamer()) {
        throw std::runtime_error("EO_TxThread failed to initialize GStreamer.");
    }
    running_.store(true);
    th_ = std::thread(&EO_TxThread::run, this);
    LOGI(TAG, "thread started");
    CSV_LOG_SIMPLE("EO.Tx", "THREAD_START", 0, 0,0,0,0, "");
}

void EO_TxThread::stop() {
    running_.store(false);
    cv_.notify_one(); // 대기 중이면 깨워서 종료 경로로
    LOGI(TAG, "stop requested");
}

void EO_TxThread::join() {
    if (th_.joinable()) {
        th_.join();
        LOGI(TAG, "thread joined");
        CSV_LOG_SIMPLE("EO.Tx", "THREAD_STOP", 0, 0,0,0,0, "");
    }
}

std::unique_ptr<WakeHandle> EO_TxThread::create_wake_handle() {
    return std::make_unique<WakeHandleCondVar>(cv_);
}

void EO_TxThread::wait_for_frame() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return !running_.load() || mb_.has_new(frame_seq_seen_); });
}

void EO_TxThread::push_frame_to_gst(const std::shared_ptr<EOFrameHandle>& handle) {
    if (!appsrc_ || !handle || !handle->p || !handle->p->data) return;

    // 원래 코드와 동일: 실제 stride(step) 기반 크기 사용
    const guint buffer_size =
        static_cast<guint>(handle->p->step * handle->p->height);

    // 버퍼 수명 보장을 위해 shared_ptr 복사본을 사용자 데이터로 보관
    auto* handle_copy = new std::shared_ptr<EOFrameHandle>(handle);

    GstBuffer* buffer = gst_buffer_new_wrapped_full(
        GST_MEMORY_FLAG_READONLY,
        (gpointer)handle->p->data,
        buffer_size,
        0,
        buffer_size,
        handle_copy,
        [](gpointer user_data) {
            auto* h_ptr = static_cast<std::shared_ptr<EOFrameHandle>*>(user_data);
            delete h_ptr;
        });

    // ns 단위 pts/duration (fps 기준)
    GST_BUFFER_PTS(buffer)      = handle->ts;
    GST_BUFFER_DTS(buffer)      = handle->ts;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, cfg_->eo_tx.fps);

    const GstFlowReturn ret = gst_app_src_push_buffer((GstAppSrc*)appsrc_, buffer);
    if (ret != GST_FLOW_OK) {
        // ❗ 전송 실패는 콘솔로만 알림. CSV에는 남기지 않음(요청사항).
        LOGE(TAG, "gst_app_src_push_buffer failed: %d", (int)ret);
    } else {
        LOGD(TAG, "pushed seq=%u bytes=%u", handle->seq, buffer_size);
    }
}

void EO_TxThread::run() {
    while (running_.load()) {
        wait_for_frame();
        if (!running_.load()) break;

        if (auto handle_opt = mb_.exchange(nullptr)) {
            const auto& handle = *handle_opt;
            double loop_ms = 0.0;
            CSV_LOG_SIMPLE("EO.Tx", "LOOP_BEGIN", handle->seq, 0,0,0,0, "");
            {
                ScopedTimerMs t(loop_ms);
                push_frame_to_gst(handle);
            }
            CSV_LOG_SIMPLE("EO.Tx", "LOOP_END", handle->seq, loop_ms, 0,0,0, "");
        }
        frame_seq_seen_ = mb_.latest_seq();
    }
    LOGI(TAG, "run() exit");
}

} // namespace flir

// EO_TxThread.cpp (TL 기반, 공통 CSV 포맷)
#include "threads_includes/EO_TxThread.hpp"

#include <chrono>
#include <sstream>
#include <stdexcept>

#include "util/telemetry.hpp"   // CSV_LOG_TL
#include "util/common_log.hpp"  // LOGI/LOGD/LOGE
#include "util/time_util.hpp"   // now_us_steady()

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

    // THREAD_START 이벤트
    CSV_LOG_TL("EO.Tx",
               0,       // seq
               0,0,0,0,
               0,
               "THREAD_START");
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

        // THREAD_STOP 이벤트
        CSV_LOG_TL("EO.Tx",
                   0,
                   0,0,0,0,
                   0,
                   "THREAD_STOP");
    }
}

std::unique_ptr<WakeHandle> EO_TxThread::create_wake_handle() {
    return std::make_unique<WakeHandleCondVar>(cv_);
}

void EO_TxThread::wait_for_frame() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return !running_.load() || mb_.has_new(frame_seq_seen_); });
}

// push_frame_to_gst: 한 프레임을 GStreamer로 밀어넣고 성공/실패 반환
bool EO_TxThread::push_frame_to_gst(const std::shared_ptr<EOFrameHandle>& handle) {
    if (!appsrc_ || !handle || !handle->p || !handle->p->data) return false;

    const guint buffer_size = static_cast<guint>(handle->p->step * handle->p->height);
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

    GST_BUFFER_PTS(buffer)      = handle->ts;
    GST_BUFFER_DTS(buffer)      = handle->ts;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, cfg_->eo_tx.fps);

    const GstFlowReturn ret = gst_app_src_push_buffer((GstAppSrc*)appsrc_, buffer);
    if (ret != GST_FLOW_OK) {
        LOGE(TAG, "gst_app_src_push_buffer failed: %d", (int)ret);
        return false;
    }
    return true;
}

void EO_TxThread::run() {
    uint64_t total_frames = 0;
    uint64_t sec_frames   = 0;
    uint64_t sec_bytes    = 0;
    auto     t0           = std::chrono::steady_clock::now();

    while (running_.load()) {
        wait_for_frame();
        if (!running_.load()) break;

        if (auto handle_opt = mb_.exchange(nullptr)) {
            const auto& handle = *handle_opt;

            // ── 타임라인 측정 ──
            std::uint64_t t0_us = now_us_steady();
            std::uint64_t t1_us = 0;
            std::uint64_t t2_us = 0;
            std::uint64_t t3_us = 0;
            std::string   note;

            const bool ok = push_frame_to_gst(handle);
            t1_us = now_us_steady();
            t2_us = t1_us; // 현재는 push 외에 추가 단계 없음

            if (ok) {
                ++total_frames;
                ++sec_frames;
                sec_bytes += static_cast<uint64_t>(handle->p->step) * handle->p->height;
                note = "PUSH_OK";
            } else {
                note = "PUSH_FAIL";
            }

            frame_seq_seen_ = mb_.latest_seq();

            // 1초 주기 로그
            auto now = std::chrono::steady_clock::now();
            if (now - t0 >= std::chrono::seconds(1)) {
                LOGI(TAG, "tx stats: fps=%llu, bytes=%llu, total=%llu",
                     (unsigned long long)sec_frames,
                     (unsigned long long)sec_bytes,
                     (unsigned long long)total_frames);
                sec_frames = 0;
                sec_bytes  = 0;
                t0 = now;
            }

            t3_us = now_us_steady();

            // 프레임 하나에 대해 딱 1줄 기록
            CSV_LOG_TL("EO.Tx",
                       handle->seq,
                       t0_us, t1_us, t2_us, t3_us,
                       0,       // t_total_us 자동 계산 (마지막 tN - t0)
                       note);
        }
    }
    LOGI(TAG, "run() exit, total_frames=%llu", (unsigned long long)total_frames);
}

} // namespace flir

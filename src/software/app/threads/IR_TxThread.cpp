#include "threads_includes/IR_TxThread.hpp"
#include "ipc/wake_condvar.hpp"
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <cstring>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// ★ 공용 로거 & CSV
#include "util/common_log.hpp"
#include "util/csv_sink.hpp"
#include "util/time_util.hpp"

namespace flir {

namespace { constexpr const char* TAG = "IR_Tx"; }

IR_TxThread::IR_TxThread(std::string name, SpscMailbox<std::shared_ptr<IRFrameHandle>>& mb, const GstConfig& gst_config)
    : name_(std::move(name)), mb_(mb), gst_config_(gst_config) {}

IR_TxThread::IR_TxThread(std::string name, SpscMailbox<std::shared_ptr<IRFrameHandle>>& mb)
    : name_(std::move(name)), mb_(mb), gst_config_(GstConfig{}) {}

IR_TxThread::~IR_TxThread() {
    stop();

    // EOS 전달 후 정리
    if (pipeline_ && appsrc_) {
        gst_app_src_end_of_stream((GstAppSrc*)appsrc_);
    }
    join();

    if (pipeline_) {
        GstBus* bus = gst_element_get_bus(pipeline_);
        if (bus) {
            gst_bus_timed_pop_filtered(bus, 500 * GST_MSECOND,
                (GstMessageType)(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
            gst_object_unref(bus);
        }
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        (void)gst_element_get_state(pipeline_, nullptr, nullptr, GST_SECOND);
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        appsrc_   = nullptr;
    }
}

bool IR_TxThread::initialize_gstreamer() {
    static bool gst_initialized = false;
    if (!gst_initialized) {
        gst_init(nullptr, nullptr);
        gst_initialized = true;
        LOGI(TAG, "gst_init()");
    }

    // NOTE: 막힘 대비용 queue를 두고 싶다면 아래처럼 appsrc 뒤에 추가:
    //   "... appsrc ... ! queue max-size-buffers=8 leaky=downstream ! udpsink ..."
    // 지금은 동작 보존을 위해 기존 파이프라인 유지.
    std::stringstream ss;
    ss << "appsrc name=ir_appsrc is-live=true do-timestamp=true block=true format=TIME "
       << "caps=video/x-raw,format=GRAY16_LE,width=" << gst_config_.width
       << ",height=" << gst_config_.height
       << ",framerate=" << gst_config_.fps << "/1 ! "
       << "udpsink host=" << gst_config_.pc_ip
       << " port=" << gst_config_.port
       << " sync=false async=false";

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

    LOGI(TAG, "init done (dst=%s:%d %dx%d@%dfps)",
         gst_config_.pc_ip.c_str(), gst_config_.port,
         gst_config_.width, gst_config_.height, gst_config_.fps);
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
    CSV_LOG_SIMPLE("IR.Tx", "THREAD_START", 0, 0,0,0,0, "");
}

void IR_TxThread::stop() { running_.store(false); cv_.notify_one(); LOGI(TAG, "stop requested"); }
void IR_TxThread::join() { if (th_.joinable()) { th_.join(); LOGI(TAG, "thread joined"); CSV_LOG_SIMPLE("IR.Tx", "THREAD_STOP", 0, 0,0,0,0, ""); } }

std::unique_ptr<WakeHandle> IR_TxThread::create_wake_handle() {
    return std::make_unique<WakeHandleCondVar>(cv_);
}

void IR_TxThread::wait_for_frame() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return !running_.load() || mb_.has_new(frame_seq_seen_); });
}

// stride(=step) 안전 복사: 패딩이 있을 수 있으므로 행단위로 채워 넣는다.
static inline void copy_ir_to_contiguous(uint8_t* dst, const IRFrame16& f, int w, int h) {
    const int want_row_bytes = w * 2;
    const int src_row_bytes  = f.step ? f.step : want_row_bytes;
    const uint8_t* src = reinterpret_cast<const uint8_t*>(f.data);
    for (int y = 0; y < h; ++y) {
        std::memcpy(dst + y * want_row_bytes, src + y * src_row_bytes, want_row_bytes);
    }
}

void IR_TxThread::push_frame_to_gst(const std::shared_ptr<IRFrameHandle>& handle) {
    if (!handle || !handle->p || !handle->p->data) return;

    const IRFrame16& f = *handle->p;

    // 크기 불일치 시 경고(안전)
    if (f.width != gst_config_.width || f.height != gst_config_.height) {
        LOGW(TAG, "frame size mismatch: got %dx%d, expected %dx%d",
             f.width, f.height, gst_config_.width, gst_config_.height);
    }

    const int num_pixels = gst_config_.width * gst_config_.height;
    const size_t payload_bytes = static_cast<size_t>(num_pixels) * sizeof(uint16_t);

    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, payload_bytes, nullptr);
    if (!buffer) { LOGE(TAG, "gst_buffer_new_allocate failed"); return; }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        LOGE(TAG, "gst_buffer_map failed");
        gst_buffer_unref(buffer);
        return;
    }

    // 패딩 고려하여 행단위 복사
    copy_ir_to_contiguous(map.data, f, gst_config_.width, gst_config_.height);
    gst_buffer_unmap(buffer, &map);

    GST_BUFFER_PTS(buffer)      = handle->ts;
    GST_BUFFER_DTS(buffer)      = handle->ts;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, gst_config_.fps);

    const GstFlowReturn ret = gst_app_src_push_buffer((GstAppSrc*)appsrc_, buffer);
    if (ret != GST_FLOW_OK) {
        // 전송 실패 카운트는 run 루프에서 집계
        throw std::runtime_error("gst_app_src_push_buffer failed");
    }
}

void IR_TxThread::run() {
    uint64_t total_frames = 0;
    uint64_t sec_frames   = 0;
    uint64_t sec_bytes    = 0;
    uint64_t sec_fail     = 0;
    auto     t0           = std::chrono::steady_clock::now();

    LOGI(TAG, "streaming → %s:%d", gst_config_.pc_ip.c_str(), gst_config_.port);

    while (running_.load()) {
        wait_for_frame();
        if (!running_.load()) break;

        if (auto handle_opt = mb_.exchange(nullptr)) {
            const auto& handle = *handle_opt;

            // CSV로는 루프 시간 상세 기록
            double loop_ms = 0.0;
            CSV_LOG_SIMPLE("IR.Tx", "LOOP_BEGIN", handle->seq, 0,0,0,0, "");
            try {
                ScopedTimerMs timer(loop_ms);
                push_frame_to_gst(handle);
            } catch (const std::exception& e) {
                ++sec_fail;
                CSV_LOG_SIMPLE("IR.Tx", "PUSH_FAIL", handle->seq, 0,0,0,0, e.what());
            }
            CSV_LOG_SIMPLE("IR.Tx", "LOOP_END", handle->seq, loop_ms, 0,0,0, "");

            // 집계
            ++total_frames;
            ++sec_frames;
            sec_bytes += static_cast<uint64_t>(gst_config_.width) * gst_config_.height * 2;
        }

        frame_seq_seen_ = mb_.latest_seq();

        // 1초 주기 로그
        auto now = std::chrono::steady_clock::now();
        if (now - t0 >= std::chrono::seconds(1)) {
            LOGI(TAG, "tx stats: fps=%llu, bytes=%llu, push_fail=%llu, total=%llu",
                 (unsigned long long)sec_frames,
                 (unsigned long long)sec_bytes,
                 (unsigned long long)sec_fail,
                 (unsigned long long)total_frames);
            sec_frames = sec_bytes = sec_fail = 0;
            t0 = now;
        }
    }

    LOGI(TAG, "stopped: total_frames=%llu", (unsigned long long)total_frames);
}

} // namespace flir

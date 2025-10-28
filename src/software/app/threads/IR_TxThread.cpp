// IR_TxThread.cpp  (CSV unified, no per-IPC CSV logs)
#include "threads_includes/IR_TxThread.hpp"

#include <sstream>
#include <chrono>
#include <cstring>
#include <stdexcept>

#include "util/common_log.hpp"   // LOGI/LOGW/...
#include "util/time_util.hpp"    // ScopedTimerMs
#include "util/csv_sink.hpp"     // CSV_LOG_SIMPLE
#include "util/telemetry.hpp"

namespace flir {

namespace {
constexpr const char* TAG = "IR_Tx";
std::once_flag g_gst_once_ir;

inline void ensure_gst_init_once() {
    std::call_once(g_gst_once_ir, []{
        int argc = 0; char** argv = nullptr;
        gst_init(&argc, &argv);
    });
}

// bitDepth → GStreamer caps용 포맷 문자열, 바이트/픽셀 반환
inline std::pair<const char*, int> bitdepth_to_caps(int bitDepth) {
    if (bitDepth <= 8)  return { "GRAY8",     1 };
    return { "GRAY16_LE", 2 }; // 14bit도 16LE로 실어보냄
}
} // namespace

IR_TxThread::IR_TxThread(std::string name,
                         AppConfigPtr cfg,
                         SpscMailbox<std::shared_ptr<IRFrameHandle>>& mb,
                         WakeHandle& wake)
    : name_(std::move(name)), cfg_(std::move(cfg)), mb_(mb), wake_(wake)
{}

IR_TxThread::~IR_TxThread() {
    stop();
    join();
    teardown_pipeline();
}

void IR_TxThread::start() {
    if (running_.exchange(true)) return;

    ensure_gst_init_once();
    if (!init_pipeline()) {
        running_.store(false);
        throw std::runtime_error("IR pipeline init failed");
    }
    th_ = std::thread(&IR_TxThread::run, this);
    LOGI(TAG, "thread started");
    CSV_LOG_SIMPLE("IR.Tx", "THREAD_START", 0, 0,0,0,0, "");
}

void IR_TxThread::stop() {
    if (!running_.load()) return;
    running_.store(false);
    // 필요 시 wake_.signal(); 추가 가능
}

void IR_TxThread::join() {
    if (th_.joinable()) {
        th_.join();
        LOGI(TAG, "thread joined");
        CSV_LOG_SIMPLE("IR.Tx", "THREAD_STOP", 0, 0,0,0,0, "");
    }
}

bool IR_TxThread::init_pipeline() {
    teardown_pipeline(); // 방어적

    const auto& c = cfg_->ir_tx;
    const auto [fmt_caps, bpp] = bitdepth_to_caps(c.bitDepth);

    // appsrc(raw gray) → udpsink
    std::ostringstream ss;
    ss << "appsrc name=ir_appsrc is-live=true do-timestamp=true block=false "
       << "! video/x-raw,format=" << fmt_caps
       << ",width=" << c.frame.width
       << ",height=" << c.frame.height
       << ",framerate=" << c.fps << "/1 "
       << "! queue max-size-buffers=8 max-size-time=0 leaky=downstream "
       << "! udpsink host=" << c.dst.ip
       << " port=" << c.dst.port;

    const auto pipeline_str = ss.str();
    LOGI(TAG, "IR pipeline: %s", pipeline_str.c_str());

    GError* err = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &err);
    if (!pipeline_) {
        if (err) { LOGE(TAG, "gst_parse_launch: %s", err->message); g_error_free(err); }
        return false;
    }

    appsrc_ = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipeline_), "ir_appsrc"));
    if (!appsrc_) {
        LOGE(TAG, "appsrc not found");
        gst_object_unref(pipeline_); pipeline_ = nullptr;
        return false;
    }

    g_object_set(G_OBJECT(appsrc_),
                 "format",     GST_FORMAT_TIME,
                 "is-live",    TRUE,
                 "block",      FALSE,
                 nullptr);

    const auto st = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (st == GST_STATE_CHANGE_FAILURE) {
        LOGE(TAG, "set_state(PLAYING) failed");
        gst_object_unref(appsrc_); appsrc_ = nullptr;
        gst_object_unref(pipeline_); pipeline_ = nullptr;
        return false;
    }

    frame_seq_seen_ = mb_.latest_seq();
    LOGI(TAG, "IR_TxThread init done (dst=%s:%u %dx%d@%dfps bpp=%d)",
         c.dst.ip.c_str(), c.dst.port, c.frame.width, c.frame.height, c.fps, bpp);
    return true;
}

void IR_TxThread::teardown_pipeline() {
    if (pipeline_) {
        gst_element_send_event(pipeline_, gst_event_new_eos());
        gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (appsrc_) { gst_object_unref(appsrc_); appsrc_ = nullptr; }
    if (pipeline_) { gst_object_unref(pipeline_); pipeline_ = nullptr; }
}

void IR_TxThread::wait_for_frame() {
    // WakeHandle 구체 타입 의존 없이 latest_seq() 폴링
    while (running_.load()) {
        const auto latest = mb_.latest_seq();
        if (latest != frame_seq_seen_) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void IR_TxThread::push_frame_to_gst(const std::shared_ptr<IRFrameHandle>& handle) {
    if (!appsrc_ || !handle || !handle->p || !handle->p->data) return;

    const auto& c = cfg_->ir_tx;
    const auto [fmt_caps, bpp] = bitdepth_to_caps(c.bitDepth);

    const int w = c.frame.width;
    const int h = c.frame.height;
    const size_t row_bytes = static_cast<size_t>(w) * bpp;
    const size_t total     = row_bytes * h;

    // FrameGRAY가 연속 메모리라고 가정(보통 Raw16 조립 결과)
    auto* sp_copy = new std::shared_ptr<IRFrameHandle>(handle);

    GstBuffer* buffer = gst_buffer_new_wrapped_full(
        GST_MEMORY_FLAG_READONLY,
        (gpointer)handle->p->data,
        total,
        0,
        total,
        sp_copy,
        [](gpointer user) {
            auto* p = static_cast<std::shared_ptr<IRFrameHandle>*>(user);
            delete p; // shared_ptr 파괴
        });

    static GstClockTime idx = 0;
    const GstClockTime duration = gst_util_uint64_scale_int(1, GST_SECOND, c.fps);
    const GstClockTime pts      = idx * duration;
    GST_BUFFER_PTS(buffer)      = pts;
    GST_BUFFER_DTS(buffer)      = pts;
    GST_BUFFER_DURATION(buffer) = duration;
    ++idx;

    const auto flow = gst_app_src_push_buffer(appsrc_, buffer);
    if (flow != GST_FLOW_OK) {
        // CSV에는 남기지 않음(요청사항). 콘솔로만 경고.
        LOGW(TAG, "gst_app_src_push_buffer flow=%d", flow);
    }
}

void IR_TxThread::run() {
    while (running_.load()) {
        wait_for_frame();
        if (!running_.load()) break;

        if (auto hopt = mb_.exchange(nullptr)) {
            const auto& h = *hopt;
            double loop_ms = 0.0;
            CSV_LOG_SIMPLE("IR.Tx", "LOOP_BEGIN", h->seq, 0,0,0,0, "");
            {
                ScopedTimerMs t(loop_ms);
                push_frame_to_gst(h);
            }
            CSV_LOG_SIMPLE("IR.Tx", "LOOP_END",   h->seq, loop_ms, 0,0,0, "");
        }
        frame_seq_seen_ = mb_.latest_seq();
    }
    LOGI(TAG, "run() exit");
}

} // namespace flir

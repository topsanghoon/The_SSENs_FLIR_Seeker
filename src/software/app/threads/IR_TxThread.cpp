// IR_TxThread.cpp (merged: safety fixes + GRAY16->GRAY8 + JPEG pipeline)
#include "threads_includes/IR_TxThread.hpp"
#include "ipc/wake_condvar.hpp"
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <cstring>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// OpenCV for 16->8 normalization
#include <opencv2/opencv.hpp>

// 우리 IRMatHandle 타입 체크용 (수명 보장)
#include "threads_includes/IR_CaptureThread.hpp"  // IRMatHandle

// 공용 로그/CSV
#include "util/common_log.hpp"
#include "util/csv_sink.hpp"
#include "util/time_util.hpp"

namespace flir {

namespace { constexpr const char* TAG = "IR_Tx"; }

// 16U(열상) → 8U 정규화 (7000~10000 → 0~255)
static inline void normalize16_to8(const IRFrame16& f, cv::Mat& out8,
                                   double minVal = 7000.0, double maxVal = 10000.0)
{
    const int w = f.width, h = f.height;
    const size_t src_step = (f.step ? f.step : w * sizeof(uint16_t));
    cv::Mat src16(h, w, CV_16UC1, const_cast<uint16_t*>(f.data), src_step);

    const double scale = 255.0 / (maxVal - minVal);
    const double delta = -minVal * scale;

    out8.create(h, w, CV_8UC1);
    src16.convertTo(out8, CV_8U, scale, delta);
}

IR_TxThread::IR_TxThread(std::string name, SpscMailbox<std::shared_ptr<IRFrameHandle>>& mb, const GstConfig& gst_config)
    : name_(std::move(name)), mb_(mb), gst_config_(gst_config) {}

IR_TxThread::IR_TxThread(std::string name, SpscMailbox<std::shared_ptr<IRFrameHandle>>& mb)
    : name_(std::move(name)), mb_(mb), gst_config_(GstConfig{}) {}

IR_TxThread::~IR_TxThread() {
    stop();

    // EOS를 먼저 appsrc에 알림
    if (pipeline_ && appsrc_) {
        gst_app_src_end_of_stream((GstAppSrc*)appsrc_);
    }
    join();

    if (pipeline_) {
        // EOS/ERROR 한 번 받아서 정리 기회 제공
        if (GstBus* bus = gst_element_get_bus(pipeline_)) {
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

    // 입력 GRAY8 → jpegenc → udpsink (수신 WPF와 합의된 파이프라인)
    std::stringstream ss;
    ss << "appsrc name=ir_appsrc is-live=true do-timestamp=true block=true format=TIME "
       << "caps=video/x-raw,format=GRAY8,width=" << gst_config_.width
       << ",height=" << gst_config_.height
       << ",framerate=" << gst_config_.fps << "/1 ! "
       << "videoscale ! video/x-raw,width=80,height=60 ! "
       << "videoconvert ! video/x-raw,format=I420 ! "
       << "jpegenc quality=" << 95 << " ! "
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

    LOGI(TAG, "init done (dst=%s:%d %dx%d@%dfps, fmt=GRAY8)",
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

void IR_TxThread::stop() {
    running_.store(false);
    cv_.notify_one();
    LOGI(TAG, "stop requested");
}

void IR_TxThread::join() {
    if (th_.joinable()) {
        th_.join();
        LOGI(TAG, "thread joined");
        CSV_LOG_SIMPLE("IR.Tx", "THREAD_STOP", 0, 0,0,0,0, "");
    }
}

std::unique_ptr<WakeHandle> IR_TxThread::create_wake_handle() {
    return std::make_unique<WakeHandleCondVar>(cv_);
}

void IR_TxThread::wait_for_frame() {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&] { return !running_.load() || mb_.has_new(frame_seq_seen_); });
}

// 안전한 프레임 푸시(세그폴트 방지) : 16U→8U 정규화 후 GRAY8 전송
void IR_TxThread::push_frame_to_gst(const std::shared_ptr<IRFrameHandle>& handle) {
    if (!handle) {
        LOGE(TAG, "Null frame handle");
        return;
    }
    
    if (!handle->p) {
        LOGE(TAG, "Null frame pointer in handle");
        return;
    }
    
    if (!handle->p->data) {
        LOGE(TAG, "Null data pointer in frame");
        return;
    }
    
    auto* mat_handle = dynamic_cast<IRMatHandle*>(handle.get());
    if (mat_handle) {
        if (!mat_handle->keep || mat_handle->keep->empty()) {
            LOGE(TAG, "Invalid cv::Mat in frame handle");
            return;
        }
        
        if (mat_handle->keep->data != reinterpret_cast<uint8_t*>(handle->p->data)) {
            LOGE(TAG, "Data pointer mismatch - possible use-after-free");
            return;
        }
    }

    const IRFrame16& f = *handle->p;

    std::shared_ptr<cv::Mat> mat_keeper;
    const uint16_t* src16 = reinterpret_cast<const uint16_t*>(f.data);
    
    if (mat_handle && mat_handle->keep) {
        mat_keeper = mat_handle->keep;
        
        if (mat_keeper->empty() || !mat_keeper->data) {
            LOGE(TAG, "cv::Mat became invalid before memcpy");
            return;
        }
        
        if (mat_keeper->data != reinterpret_cast<uint8_t*>(handle->p->data)) {
            LOGE(TAG, "Data pointer changed before memcpy");
            return;
        }
        
        src16 = reinterpret_cast<const uint16_t*>(mat_keeper->data);
    }

    // 16U → 8U 정규화
    cv::Mat gray8;
    normalize16_to8(f, gray8);  // 7000~10000 → 0~255

    const size_t need_bytes = static_cast<size_t>(gst_config_.width) * gst_config_.height;
    if (static_cast<size_t>(gray8.total()) != need_bytes) {
        LOGE(TAG, "size mismatch: gray8=%zu, caps=%zu (drop frame)",
             (size_t)gray8.total(), need_bytes);
        return;
    }

    GstBuffer* buffer = gst_buffer_new_allocate(nullptr, need_bytes, nullptr);
    if (!buffer) { 
        LOGE(TAG, "gst_buffer_new_allocate failed"); 
        return; 
    }

    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
        LOGE(TAG, "gst_buffer_map failed");
        gst_buffer_unref(buffer);
        return;
    }
    
    if (map.size < need_bytes) {
        LOGE(TAG, "GStreamer buffer too small: needed=%zu got=%zu", need_bytes, (size_t)map.size);
        gst_buffer_unmap(buffer, &map);
        gst_buffer_unref(buffer);
        return;
    }

    if (!gray8.data) {
        LOGE(TAG, "Source pointer is null before memcpy");
        gst_buffer_unmap(buffer, &map);
        gst_buffer_unref(buffer);
        return;
    }

    std::memcpy(map.data, gray8.data, need_bytes);
    gst_buffer_unmap(buffer, &map);

    GST_BUFFER_PTS(buffer)      = handle->ts;
    GST_BUFFER_DTS(buffer)      = handle->ts;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, gst_config_.fps);

    const GstFlowReturn ret = gst_app_src_push_buffer((GstAppSrc*)appsrc_, buffer);
    if (ret != GST_FLOW_OK) {
        LOGE(TAG, "gst_app_src_push_buffer failed (%d)", (int)ret);
        gst_buffer_unref(buffer);
    }
}

void IR_TxThread::run() {
    uint64_t total_frames = 0, sec_frames = 0, sec_bytes = 0, sec_fail = 0;
    auto t0 = std::chrono::steady_clock::now();

    LOGI(TAG, "streaming → %s:%d (GRAY8)",
         gst_config_.pc_ip.c_str(), gst_config_.port);

    while (running_.load()) {
        wait_for_frame();
        if (!running_.load()) break;

        if (auto handle_opt = mb_.exchange(nullptr)) {
            const auto& handle = *handle_opt;

            double loop_ms = 0.0;
            CSV_LOG_SIMPLE("IR.Tx", "LOOP_BEGIN", handle ? handle->seq : 0, 0,0,0,0, "");
            try {
                flir::ScopedTimerMs timer(loop_ms);
                push_frame_to_gst(handle);
            } catch (const std::exception& e) {
                ++sec_fail;
                CSV_LOG_SIMPLE("IR.Tx", "PUSH_FAIL", handle ? handle->seq : 0, 0,0,0,0, e.what());
            }
            CSV_LOG_SIMPLE("IR.Tx", "LOOP_END", handle ? handle->seq : 0, loop_ms, 0,0,0, "");

            ++total_frames;
            ++sec_frames;
            sec_bytes += static_cast<uint64_t>(gst_config_.width) * gst_config_.height;
        }

        frame_seq_seen_ = mb_.latest_seq();

        // 1초 주기 통계
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
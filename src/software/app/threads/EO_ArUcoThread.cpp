#include "threads_includes/EO_ArUcoThread.hpp"
#include "components/includes/CsvLoggerAru.hpp"  // 구체 로거
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <iostream>

namespace flir {

static std::mutex              g_m_eo;
static std::condition_variable g_cv_eo;
using clock_t = std::chrono::steady_clock;

EO_ArUcoThread::EO_ArUcoThread(SpscMailbox<std::shared_ptr<EOFrameHandle>>& eo_mb,
                               IArucoPreprocessor&       preproc,
                               IArucoDetector&           detector,
                               IEventBus&                bus,
                               CsvLoggerAru&             logger)
: eo_mb_(eo_mb), preproc_(preproc), detector_(detector), bus_(bus), log_(logger) {}

void EO_ArUcoThread::start(){ 
    running_.store(true); 
    th_ = std::thread(&EO_ArUcoThread::run, this); 
}
void EO_ArUcoThread::stop(){ 
    running_.store(false); 
    g_cv_eo.notify_all(); 
}
void EO_ArUcoThread::join(){ 
    if (th_.joinable()) th_.join(); 
}

void EO_ArUcoThread::onFrameArrived(std::shared_ptr<EOFrameHandle> h){ eo_mb_.push(h); g_cv_eo.notify_one(); }

void EO_ArUcoThread::run() {
    while (running_.load()) {
        wait_until_ready();
        if (!running_.load()) break;
        if (!eo_mb_.has_new(frame_seq_seen_)) continue;
        if (auto h_opt = eo_mb_.exchange(nullptr)) {
            on_frame(h_opt.value());
        }
    }
}

void EO_ArUcoThread::wait_until_ready() {
    std::unique_lock<std::mutex> lk(g_m_eo);
    g_cv_eo.wait(lk, [&]{
        return !running_.load() || eo_mb_.latest_seq() > frame_seq_seen_;
    });
}

void EO_ArUcoThread::on_frame(const std::shared_ptr<EOFrameHandle>& h) {
    frame_seq_seen_ = h->seq;

    auto t0 = clock_t::now();

    cv::Mat pf;
    preproc_.run(*h, pf);                   // BGR8 → GRAY8

    auto detections = detector_.detect(pf);

    const double ms = std::chrono::duration<double,std::milli>(clock_t::now() - t0).count();

    if (!detections.empty()) {
        for (auto& d : detections) {
            emit_aruco(d.id, d.corners, d.bbox, h->ts, h->seq);
        }
        log_.marker_found(h->seq, detections.size(), ms);
    } else {
        log_.marker_lost(h->seq, ms);
    }
}

void EO_ArUcoThread::emit_aruco(int id,
                                const std::array<cv::Point2f,4>& corners,
                                const cv::Rect2f& box,
                                uint64_t ts_ns, uint32_t /*frame_seq*/) {
    ArucoEvent a{ id, corners, box, ts_ns };
    Event ev{ EventType::Aruco, a };
    bus_.push(ev, Topic::Aruco);
}

void EO_ArUcoThread::log_debug(const std::string& msg) {
    std::cout << "[EO_ArUcoThread] " << msg << std::endl;
}

} // namespace flir

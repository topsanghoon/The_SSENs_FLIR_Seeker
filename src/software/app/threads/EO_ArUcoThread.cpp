#include "threads_includes/EO_ArUcoThread.hpp"
#include <condition_variable>
#include <mutex>
#include <chrono>

namespace flir {

// 실제 프로젝트에선 ThreadManager가 CV를 소유할 수도 있음.
// 여기선 스켈레톤으로 내부 정적 CV 사용.
static std::mutex              g_m_eo;
static std::condition_variable g_cv_eo;

using clock_t = std::chrono::steady_clock;

EO_ArUcoThread::EO_ArUcoThread(SpscMailbox<EOFrameHandle>& eo_mb,
                               IArucoPreprocessor&       preproc,
                               IArucoDetector&           detector,
                               IEventBus&                bus,
                               CsvLoggerAru              logger)
: eo_mb_(eo_mb)
, preproc_(preproc)
, detector_(detector)
, bus_(bus)
, log_(std::move(logger))
{}

void EO_ArUcoThread::start() {
    running_.store(true);
    th_ = std::thread(&EO_ArUcoThread::run, this);
}

void EO_ArUcoThread::stop() {
    running_.store(false);
    g_cv_eo.notify_all(); // 잠자고 있으면 깨워서 종료
}

void EO_ArUcoThread::join() {
    if (th_.joinable()) th_.join();
}

void EO_ArUcoThread::onFrameArrived(EOFrameHandle h) {
    // EO_CaptureThread가 프레임을 밀어넣고 깨우고 싶을 때 호출(선택)
    eo_mb_.push(h);
    g_cv_eo.notify_one();
}

void EO_ArUcoThread::run() {
    while (running_.load()) {
        wait_until_ready();
        if (!running_.load()) break;

        // 프레임이 실제로 새로 왔는지 확인
        if (!eo_mb_.has_new(frame_seq_seen_)) continue;

        if (auto h = eo_mb_.exchange(nullptr)) {
            EOFrameHandle fh = *h;                 // 복사/이동 선택
            on_frame(fh);                        // 검출
            fh.release();                        // ref--
        }
    }
}

void EO_ArUcoThread::wait_until_ready() {
    std::unique_lock<std::mutex> lk(g_m_eo);
    g_cv_eo.wait(lk, [&]{
        return !running_.load() ||
               eo_mb_.latest_seq() > frame_seq_seen_;
    });
}

void EO_ArUcoThread::on_frame(EOFrameHandle& h) {
    frame_seq_seen_ = h.seq;

    // 타이머 시작
    auto t0 = clock_t::now();

    // 1) 전처리 (예: RGB → GRAY8, 블러/이퀄라이즈 등)
    cv::Mat pf;
    preproc_.run(h, pf);

    // 2) 검출
    auto detections = detector_.detect(pf);

    // 3) 결과 발행/로깅
    auto ms = std::chrono::duration<double, std::milli>(clock_t::now() - t0).count();

    if (!detections.empty()) {
        for (auto& d : detections) {
            emit_aruco(d.id, d.bbox, h.ts, h.seq);
        }
        log_.marker_found(h.seq, detections.size(), ms);
    } else {
        log_.marker_lost(h.seq, ms);
        // 실패는 무발행 (요구사항)
    }
}

void EO_ArUcoThread::emit_aruco(int id, const cv::Rect2f& box, uint64_t ts_ns, uint32_t frame_seq) {
    ArucoEvent a{ id, box, /*pose*/ ts_ns };
    Event ev{ EventType::Aruco, a };
    bus_.push(ev, Topic::Aruco); // Meta_TxThread가 구독 중
}

} // namespace flir

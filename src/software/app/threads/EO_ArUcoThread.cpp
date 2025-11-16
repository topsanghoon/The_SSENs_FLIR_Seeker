// EO_ArUcoThread.cpp (CSV-unified, no per-module CsvLogger)
#include "threads_includes/EO_ArUcoThread.hpp"
#include "util/telemetry.hpp"
#include "util/time_util.hpp"                      // now_ms_epoch(), ScopedTimerMs
#include "util/common_log.hpp"                     // LOGD/LOGI/...
#include "util/csv_sink.hpp"                       // ✅ 단일 CSV 싱크
#include "phase_gate.hpp"

#include <condition_variable>
#include <mutex>
#include <chrono>
#include <array>
#include <optional>
#include <vector>
#include <algorithm>
#include <iostream>

namespace flir {

using clock_t = std::chrono::steady_clock;

namespace {
constexpr const char* kTAG = "EO.Aruco";

// 프레임 단위 상세 로그를 켤지(많이 시끄러울 수 있음)
constexpr bool kVerbosePerFrameLog = false;
} // namespace

EO_ArUcoThread::EO_ArUcoThread(SpscMailbox<std::shared_ptr<EOFrameHandle>>& eo_mb,
                               IArucoPreprocessor&       preproc,
                               IArucoDetector&           detector,
                               IEventBus&                bus
                               /* CsvLoggerAru& logger 제거 */)
: eo_mb_(eo_mb), preproc_(preproc), detector_(detector), bus_(bus) {}

void EO_ArUcoThread::start(){
    running_.store(true);
    LOGI(kTAG, "start()");
    CSV_LOG_SIMPLE("EO.Aruco", "THREAD_START", 0, 0,0,0,0, "");
    th_ = std::thread(&EO_ArUcoThread::run, this);
}

void EO_ArUcoThread::stop(){
    LOGI(kTAG, "stop() requested");
    running_.store(false);
    cv_.notify_all();
}

void EO_ArUcoThread::join(){
    if (th_.joinable()){
        th_.join();
        LOGI(kTAG, "join() done");
        CSV_LOG_SIMPLE("EO.Aruco", "THREAD_STOP", 0, 0,0,0,0, "");
    }
}


void EO_ArUcoThread::onFrameArrived(std::shared_ptr<EOFrameHandle> h){
    eo_mb_.push(std::move(h));
    cv_.notify_one();                    // ★ 전용 cv로 깨움
}

bool EO_ArUcoThread::is_big_enough(int bw, int bh) const {
    
    // 절대 기준
    bool big_abs = (bw >= cfg_.guidance.min_bbox_w) &&
                   (bh >= cfg_.guidance.min_bbox_h);

    // (선택) 비율 기준
    bool big_frac = false;
    if (cfg_.guidance.min_bbox_frac > 0.f && cfg_.eo_w > 0 && cfg_.eo_h > 0) {
        float fw = static_cast<float>(bw) / static_cast<float>(cfg_.eo_w);
        float fh = static_cast<float>(bh) / static_cast<float>(cfg_.eo_h);
        big_frac = (std::max(fw, fh) >= cfg_.guidance.min_bbox_frac);
    }
    return big_abs || big_frac;
}

void EO_ArUcoThread::run() {
    LOGI(kTAG, "run() loop enter");

    // 주기 요약 로그용 통계
    uint64_t stat_t0_ms = now_ms_epoch();
    uint64_t last_log_ms = stat_t0_ms;
    uint32_t stat_frames = 0;
    uint32_t stat_found  = 0;
    uint32_t stat_lost   = 0;
    double   stat_sum_ms = 0.0;
    double   stat_max_ms = 0.0;
    double   stat_min_ms = 1e12;

    while (running_.load()) {
        if (!flir::eo_enabled()) { std::unique_lock<std::mutex> lk(m_); cv_.wait_for(lk, std::chrono::milliseconds(5)); continue; }
        
        wait_until_ready();
        if (!running_.load()) break;
        if (!eo_mb_.has_new(frame_seq_seen_)) continue;

        if (auto h_opt = eo_mb_.exchange(nullptr)) {
            // 옵셔널과 shared_ptr을 벗겨 실제 프레임 참조를 얻는다
            auto& sh = *h_opt;                // std::shared_ptr<EOFrameHandle>&
            const EOFrameHandle& fr = *sh;    // EOFrameHandle& → 인터페이스에 맞는 실제 객체

            frame_seq_seen_ = fr.seq;

            double t_ms_total = 0.0, t_ms_pre = 0.0, t_ms_det = 0.0;
            CSV_LOG_SIMPLE("EO.Aruco", "LOOP_BEGIN", fr.seq, 0,0,0,0, "");
            {
                ScopedTimerMs t_all(t_ms_total);

                try{

                
                // --- 전처리 ---
                cv::Mat pf;
                {
                    ScopedTimerMs t_pre(t_ms_pre);
                    preproc_.run(fr, pf);     // const EOFrameHandle& 로 전달
                }

                if (pf.empty()) {
                    CSV_LOG_SIMPLE("EO.Aruco", "PRE_EMPTY", fr.seq, 0,0,0,0, "");
                    continue;
                }

                // --- 검출 ---
                std::vector<IArucoDetector::Detection> detections;
                {
                    ScopedTimerMs t_det(t_ms_det);
                    detections = detector_.detect(pf);
                }

                const bool found = !detections.empty();

                if (found) {
                    for (auto& d : detections) {
                        if(toFindAruco == d.id) {
                            emit_aruco(d.id, d.corners, d.bbox, fr.ts, fr.seq);
                            if(is_big_enough(static_cast<int>(d.bbox.width), static_cast<int>(d.bbox.height))) {
                                toFindAruco++;
                            }
                        }
                    }
                    // 마커 발견 이벤트: v1=pre_ms, v2=det_ms, note=개수
                    CSV_LOG_SIMPLE("EO.Aruco", "MARKER_FOUND", fr.seq,
                                   t_ms_pre, t_ms_det, 0, 0,
                                   "n=" + std::to_string(detections.size()));
                } else {
                    CSV_LOG_SIMPLE("EO.Aruco", "MARKER_LOST", fr.seq,
                                   t_ms_pre, t_ms_det, 0, 0, "");
                }

                if (kVerbosePerFrameLog) {
                    LOGDs(kTAG) << "seq=" << fr.seq
                                << " ts="  << fr.ts
                                << " pre_ms=" << t_ms_pre
                                << " det_ms=" << t_ms_det
                                << " total_ms=" << t_ms_total
                                << " n=" << detections.size();
                }

                // 통계 갱신
                stat_frames++;
                stat_sum_ms += t_ms_total;
                stat_max_ms  = std::max(stat_max_ms, t_ms_total);
                stat_min_ms  = std::min(stat_min_ms, t_ms_total);
                if (found) stat_found++; else stat_lost++;
                } catch(const cv::Exception& e){
                    LOGE(kTAG, "detect exception: %s", e.what());
                    CSV_LOG_SIMPLE("EO.Aruco", "DETECT_EX", fr.seq, 0,0,0,0, e.what());
                    // 통계는 lost로 카운트하거나 skip
                    stat_lost++;
                    continue;
                }
            }
            // 세부 시간 측정치를 별도 이벤트로 남겨 두면 후처리 필터링이 쉬움
            CSV_LOG_SIMPLE("EO.Aruco", "PRE_MS",   fr.seq, t_ms_pre,   0, 0, 0, "");
            CSV_LOG_SIMPLE("EO.Aruco", "DET_MS",   fr.seq, t_ms_det,   0, 0, 0, "");
            CSV_LOG_SIMPLE("EO.Aruco", "LOOP_END", fr.seq, t_ms_total, 0, 0, 0, "");
        }

        // 1초 주기 요약 로그
        const uint64_t now_ms = now_ms_epoch();
        if (now_ms - last_log_ms >= 1000) {
            const double elapsed_s = (now_ms - last_log_ms) / 1000.0;
            const double fps       = stat_frames / std::max(1e-9, elapsed_s);
            const double avg_ms    = (stat_frames ? (stat_sum_ms / stat_frames) : 0.0);

            LOGI(kTAG,
                 "FPS=%.1f frames=%u found=%u lost=%u avg=%.2fms min=%.2fms max=%.2fms (%.2fs)",
                 fps, stat_frames, stat_found, stat_lost, avg_ms, stat_min_ms, stat_max_ms, elapsed_s);

            // 리셋
            last_log_ms = now_ms;
            stat_frames = 0;
            stat_found  = 0;
            stat_lost   = 0;
            stat_sum_ms = 0.0;
            stat_max_ms = 0.0;
            stat_min_ms = 1e12;
        }
    }

    LOGI(kTAG, "run() loop exit");
}

void EO_ArUcoThread::wait_until_ready() {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait(lk, [&]{
        const bool ready = (!running_.load() || eo_mb_.latest_seq() > frame_seq_seen_);
        return ready;
    });
}

void EO_ArUcoThread::on_frame(const std::shared_ptr<EOFrameHandle>& h) {
    // 기존 단순 처리 함수는 run()에서 직접 인라인/확장 처리로 대체.
    // 남겨두고 싶다면 여기에 공용 처리 로직을 옮겨도 됨.
    frame_seq_seen_ = h->seq;
}

void EO_ArUcoThread::emit_aruco(int id,
                                const std::array<cv::Point2f,4>& corners,
                                const cv::Rect2f& box,
                                uint64_t ts_ns, uint32_t frame_seq) {
    if (kVerbosePerFrameLog) {
        LOGDs(kTAG) << "emit id=" << id
                    << " seq=" << frame_seq
                    << " ts="  << ts_ns
                    << " box=(" << box.x << "," << box.y
                    << "," << box.width << "x" << box.height << ")";
    }

    // 이벤트 버스로 전달
    ArucoEvent a{ id, corners, box, ts_ns };
    Event ev{ EventType::Aruco, a };
    bus_.push(ev, Topic::Aruco);

}

} // namespace flir

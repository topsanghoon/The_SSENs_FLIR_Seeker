// EO_ArUcoThread.cpp (CSV-unified, TL 기반)
#include "threads_includes/EO_ArUcoThread.hpp"
#include "util/telemetry.hpp"
#include "util/time_util.hpp"                      // now_ms_epoch(), now_us_steady()
#include "util/common_log.hpp"                     // LOGD/LOGI/...
#include "phase_gate.hpp"

#include <condition_variable>
#include <mutex>
#include <chrono>
#include <array>
#include <optional>
#include <vector>
#include <algorithm>
#include <iostream>
#include <string>

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
                               IEventBus&                bus)
: eo_mb_(eo_mb)
, preproc_(preproc)
, detector_(detector)
, bus_(bus) {}

void EO_ArUcoThread::start(){
    running_.store(true);
    LOGI(kTAG, "start()");

    // 스레드 시작 이벤트
    CSV_LOG_TL("EO.Aruco",
               0,      // seq
               0,0,0,0,
               0,      // total_us (의미 없음)
               "THREAD_START");

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

        // 스레드 종료 이벤트
        CSV_LOG_TL("EO.Aruco",
                   0,
                   0,0,0,0,
                   0,
                   "THREAD_STOP");
    }
}

void EO_ArUcoThread::onFrameArrived(std::shared_ptr<EOFrameHandle> h){
    eo_mb_.push(std::move(h));
    cv_.notify_one();                    // ★ 전용 cv로 깨움
}

bool EO_ArUcoThread::is_big_enough(int bw, int bh) const {
    bool big_abs = (bw >= cfg_.guidance.min_bbox_w) &&
                   (bh >= cfg_.guidance.min_bbox_h);

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
    uint64_t stat_t0_ms  = now_ms_epoch();
    uint64_t last_log_ms = stat_t0_ms;
    uint32_t stat_frames = 0;
    uint32_t stat_found  = 0;
    uint32_t stat_lost   = 0;
    double   stat_sum_ms = 0.0;
    double   stat_max_ms = 0.0;
    double   stat_min_ms = 1e12;

    while (running_.load()) {
        if (!flir::eo_enabled()) {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait_for(lk, std::chrono::milliseconds(5));
            continue;
        }

        wait_until_ready();
        if (!running_.load()) break;
        if (!eo_mb_.has_new(frame_seq_seen_)) continue;

        if (auto h_opt = eo_mb_.exchange(nullptr)) {
            auto& sh = *h_opt;                // std::shared_ptr<EOFrameHandle>&
            const EOFrameHandle& fr = *sh;    // EOFrameHandle&
            frame_seq_seen_ = fr.seq;

            // ─────────────────────────────────────
            // 타임라인 (us) + note (1줄로 기록)
            // ─────────────────────────────────────
            uint64_t t0_us = now_us_steady();
            uint64_t t1_us = 0;
            uint64_t t2_us = 0;
            uint64_t t3_us = 0;
            std::string note;

            bool found = false;

            try {
                // --- 전처리 ---
                cv::Mat pf;
                preproc_.run(fr, pf);
                t1_us = now_us_steady();

                if (pf.empty()) {
                    note = "PRE_EMPTY";
                    // detect 안 했으므로 t2=t1, t3=t1 로 맞춤
                    t2_us = t1_us;
                    t3_us = t1_us;

                    double total_ms = (t3_us >= t0_us)
                                      ? (double)(t3_us - t0_us) / 1000.0
                                      : 0.0;

                    stat_frames++;
                    stat_lost++;
                    stat_sum_ms += total_ms;
                    stat_max_ms = std::max(stat_max_ms, total_ms);
                    stat_min_ms = std::min(stat_min_ms, total_ms);

                    CSV_LOG_TL("EO.Aruco",
                               fr.seq,
                               t0_us, t1_us, t2_us, t3_us,
                               0,          // total_us 자동계산
                               note);
                    continue;
                }

                // --- 검출 ---
                std::vector<IArucoDetector::Detection> detections;
                detections = detector_.detect(pf);
                t2_us = now_us_steady();

                found = !detections.empty();
                int temp_id;
                if (found) {
                    for (auto& d : detections) {
                        if (toFindAruco == d.id) {
                            emit_aruco(d.id, d.corners, d.bbox, fr.ts, fr.seq);
                            temp_id = d.id;
                            if (toFindAruco != 3 &&
                                is_big_enough(static_cast<int>(d.bbox.width),
                                              static_cast<int>(d.bbox.height))) {
                                toFindAruco++;
                            }
                        }
                    }

                    note = "FOUND:id=" + std::to_string(temp_id);
                } else {
                    note = "LOST";
                }

                if (kVerbosePerFrameLog) {
                    double pre_ms   = (t1_us >= t0_us)
                                      ? (double)(t1_us - t0_us) / 1000.0 : 0.0;
                    double det_ms   = (t2_us >= t1_us)
                                      ? (double)(t2_us - t1_us) / 1000.0 : 0.0;
                    double total_ms = (t2_us >= t0_us)
                                      ? (double)(t2_us - t0_us) / 1000.0 : 0.0;

                    LOGDs(kTAG) << "seq=" << fr.seq
                                << " ts="  << fr.ts
                                << " pre_ms=" << pre_ms
                                << " det_ms=" << det_ms
                                << " total_ms=" << total_ms
                                << " n=" << detections.size();
                }

                // 통계 갱신 (t3_us 를 이따 확정하고 나서)
                t3_us = now_us_steady();
                double total_ms = (t3_us >= t0_us)
                                  ? (double)(t3_us - t0_us) / 1000.0
                                  : 0.0;

                stat_frames++;
                stat_sum_ms += total_ms;
                stat_max_ms = std::max(stat_max_ms, total_ms);
                stat_min_ms = std::min(stat_min_ms, total_ms);
                if (found) stat_found++; else stat_lost++;

                CSV_LOG_TL("EO.Aruco",
                           fr.seq,
                           t0_us, t1_us, t2_us, t3_us,
                           0,          // total_us 자동 계산
                           note);
            } catch (const cv::Exception& e) {
                LOGE(kTAG, "detect exception: %s", e.what());
                t3_us = now_us_steady();

                double total_ms = (t3_us >= t0_us)
                                  ? (double)(t3_us - t0_us) / 1000.0
                                  : 0.0;

                stat_frames++;
                stat_lost++;
                stat_sum_ms += total_ms;
                stat_max_ms = std::max(stat_max_ms, total_ms);
                stat_min_ms = std::min(stat_min_ms, total_ms);

                note = std::string("DETECT_EX:") + e.what();

                CSV_LOG_TL("EO.Aruco",
                           fr.seq,
                           t0_us, t1_us, t2_us, t3_us,
                           0,
                           note);
                continue;
            }
        }

        // 1초 주기 요약 로그 (콘솔용)
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

    ArucoEvent a{ id, corners, box, ts_ns};
    Event ev{ EventType::Aruco, a };
    bus_.push(ev, Topic::Aruco);
}

} // namespace flir

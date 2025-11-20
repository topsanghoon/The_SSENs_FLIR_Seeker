// EO_ArUcoThread.cpp (CSV-unified, TL 기반 + ROI 탐지)
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

// ──────────────────────────────
// ROI 상태 (이 스레드에서만 사용)
// ──────────────────────────────
cv::Rect2f g_last_box;          // 마지막으로 본 마커 박스(전역 좌표)
bool       g_have_last_box = false;
int        g_lost_consec   = 0;

// ROI 튜닝 파라미터
constexpr int kROI_MARGIN_PX   = 40;  // 마지막 박스 주변 여유 픽셀
constexpr int kROI_MIN_W       = 40;  // ROI 최소 폭
constexpr int kROI_MIN_H       = 40;  // ROI 최소 높이
constexpr int kROI_MAX_LOST    = 10;  // 이만큼 연속으로 못 찾으면 ROI 리셋

// 이미지 크기와 마지막 박스를 기반으로 ROI를 계산
inline cv::Rect compute_roi(const cv::Size& sz)
{
    if (!g_have_last_box || g_lost_consec >= kROI_MAX_LOST) {
        // ROI 사용하지 않고 전체 프레임
        return cv::Rect(0, 0, sz.width, sz.height);
    }

    int x1 = static_cast<int>(std::floor(g_last_box.x)) - kROI_MARGIN_PX;
    int y1 = static_cast<int>(std::floor(g_last_box.y)) - kROI_MARGIN_PX;
    int x2 = static_cast<int>(std::ceil (g_last_box.x + g_last_box.width))  + kROI_MARGIN_PX;
    int y2 = static_cast<int>(std::ceil (g_last_box.y + g_last_box.height)) + kROI_MARGIN_PX;

    x1 = std::max(0, x1);
    y1 = std::max(0, y1);
    x2 = std::min(sz.width,  x2);
    y2 = std::min(sz.height, y2);

    int w = x2 - x1;
    int h = y2 - y1;

    if (w < kROI_MIN_W || h < kROI_MIN_H) {
        // 너무 작아지면 다시 전체 프레임
        return cv::Rect(0, 0, sz.width, sz.height);
    }
    return cv::Rect(x1, y1, w, h);
}

} // namespace anon

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

            bool found_any    = false;  // 아무 마커라도 찾았는지
            bool found_target = false;  // toFindAruco 와 id가 일치하는 마커를 찾았는지
            int  found_id     = -1;

            // 이번 프레임에서 사용할 ROI (전역 좌표 기준)
            cv::Rect roi_for_log;       // 로그용
            cv::Rect roi(0, 0, 0, 0);   // 실제 사용

            try {
                // --- 전처리: full frame GRAY로 만들기 ---
                cv::Mat pf;
                preproc_.run(fr, pf);
                t1_us = now_us_steady();

                if (pf.empty()) {
                    note = "PRE_EMPTY";
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

                // --- ROI 계산 ---
                cv::Size sz = pf.size();
                roi = compute_roi(sz);
                roi_for_log = roi;

                cv::Mat pf_roi = pf(roi);   // 관측 영역만 잘라서 탐지

                // --- 검출 ---
                std::vector<IArucoDetector::Detection> detections;
                detections = detector_.detect(pf_roi);
                t2_us = now_us_steady();

                found_any = !detections.empty();

                // ROI 좌표 → 전역 좌표로 보정하면서 타깃 마커 찾기
                cv::Rect2f new_last_box;      // ROI 전략용
                bool      updated_last_box = false;

                if (found_any) {
                    for (auto& d : detections) {
                        // ROI 기준 → 전역 기준으로 변환
                        std::array<cv::Point2f,4> corners_global = d.corners;
                        for (auto& p : corners_global) {
                            p.x += static_cast<float>(roi.x);
                            p.y += static_cast<float>(roi.y);
                        }
                        cv::Rect2f box_global(
                            d.bbox.x + roi.x,
                            d.bbox.y + roi.y,
                            d.bbox.width,
                            d.bbox.height
                        );

                        // ROI 기준 박스 갱신 후보 저장 (첫 마커 기준)
                        if (!updated_last_box) {
                            new_last_box      = box_global;
                            updated_last_box  = true;
                        }

                        // 우리가 찾고 있는 id와 일치하면 이벤트 발행
                        if (toFindAruco == d.id) {
                            emit_aruco(d.id, corners_global, box_global,
                                       fr.ts, fr.seq);
                            found_target = true;
                            found_id     = d.id;

                            // ROI 중심을 이 타깃 박스로 맞추도록 저장
                            new_last_box     = box_global;
                            updated_last_box = true;

                            if (toFindAruco != 3 &&
                                is_big_enough(static_cast<int>(box_global.width),
                                              static_cast<int>(box_global.height))) {
                                toFindAruco++;
                            }
                        }
                    }

                    if (found_target) {
                        note = "FOUND:id=" + std::to_string(found_id);
                    } else {
                        note = "FOUND_OTHER";  // 마커는 있으나 원하는 id는 아님
                    }

                    // ROI 상태 갱신
                    if (updated_last_box) {
                        g_last_box      = new_last_box;
                        g_have_last_box = true;
                        g_lost_consec   = 0;
                    }
                } else {
                    // 이번 프레임에서는 아무 마커도 발견 못함
                    note = "LOST";

                    if (g_have_last_box) {
                        g_lost_consec++;
                        if (g_lost_consec >= kROI_MAX_LOST) {
                            // 일정 프레임 이상 못 찾았으면 ROI 리셋 → 다음 프레임부터 전체 프레임 탐색
                            g_have_last_box = false;
                            note += ";ROI_RESET";
                        }
                    }
                }

                // ROI 정보도 note에 추가
                note += ";ROI=(" +
                        std::to_string(roi_for_log.x) + "," +
                        std::to_string(roi_for_log.y) + "," +
                        std::to_string(roi_for_log.width) + "x" +
                        std::to_string(roi_for_log.height) + ")";

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
                                << " n=" << detections.size()
                                << " roi=(" << roi_for_log.x << ","
                                            << roi_for_log.y << ","
                                            << roi_for_log.width << "x"
                                            << roi_for_log.height << ")";
                }

                // 통계 갱신
                t3_us = now_us_steady();
                double total_ms = (t3_us >= t0_us)
                                  ? (double)(t3_us - t0_us) / 1000.0
                                  : 0.0;

                stat_frames++;
                stat_sum_ms += total_ms;
                stat_max_ms = std::max(stat_max_ms, total_ms);
                stat_min_ms = std::min(stat_min_ms, total_ms);
                if (found_any) stat_found++; else stat_lost++;

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

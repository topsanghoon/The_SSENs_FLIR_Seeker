// IR_TrackThread.cpp  (TL 기반 CSV / CsvLoggerIR 제거)
#include "threads_includes/IR_TrackThread.hpp"

#include <iostream>
#include <condition_variable>
#include <mutex>
#include <string>
#include <sstream>

#include "util/telemetry.hpp"   // CSV_LOG_TL
#include "util/time_util.hpp"   // now_us_steady()
#include "util/common_log.hpp"
#include "phase_gate.hpp"

/*
종말 유도가 아니면 tracking 스레드에 데이터가 들어오지 않음으로 깨어날 일 없음.
중기 유도가 끝나면 중기 ArUco 스레드가 종료되면서 전역 변수를 바꿀것이고, 바꾸면 이후 캡쳐 과정에서 IR 프레임을
트래킹 스레드로 전송하게 되고, 그때부터 자연스럽게 동작
*/

namespace flir {

static constexpr const char* TAG = "IR.Track";


IR_TrackThread::IR_TrackThread(SpscMailbox<std::shared_ptr<IRFrameHandle>>& ir_mb,
                               SpscMailbox<UserCmd>&     click_mb,
                               ITrackerStrategy&         tracker,
                               IPreprocessor&            preproc,
                               IEventBus&                bus,
                               IRTrackConfig             cfg)
: ir_mb_(ir_mb)
, click_mb_(click_mb)
, tracker_(tracker)
, preproc_(preproc)
, bus_(bus)
, cfg_(cfg)
{}

// start, stop, join — 스레드 제어
void IR_TrackThread::start() {
    if (running_.exchange(true)) return;

    // THREAD_START 타임라인
    CSV_LOG_TL("IR.Track",
               0,      // seq
               0,0,0,0,
               0,
               "THREAD_START");

    th_ = std::thread(&IR_TrackThread::run, this);
}

void IR_TrackThread::stop() {
    if (!running_.load()) return;
    running_.store(false);
    cv_.notify_all(); // 대기 중이면 깨워서 종료 경로로
}

void IR_TrackThread::join() {
    if (th_.joinable()) {
        th_.join();

        // THREAD_STOP 타임라인
        CSV_LOG_TL("IR.Track",
                   0,
                   0,0,0,0,
                   0,
                   "THREAD_STOP");
    }
}

/*
프레임/클릭이 도착하면 메일박스에 넣고 깨움.
프레임이 와야 한 스텝이 진행된다.
*/
void IR_TrackThread::onFrameArrived(std::shared_ptr<IRFrameHandle> h) {
    ir_mb_.push(std::move(h));
    cv_.notify_one();
}

void IR_TrackThread::onClickArrived(const UserCmd& cmd) {
    click_mb_.push(cmd);
    cv_.notify_one();
}

// 메인 루프
void IR_TrackThread::run() {
    while (running_.load()) {
        if (!flir::ir_enabled()) {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait_for(lk, std::chrono::milliseconds(5));
            continue;
        }

        wait_until_ready();
        if (!running_.load()) break;

        // 1) 클릭 먼저 모두 처리
        while (click_mb_.has_new(click_mb_seq_seen_)) {
            if (auto cmd = click_mb_.exchange(nullptr)) {
                handle_click(*cmd);
            } else {
                break;
            }
        }

        // 2) IR 프레임 처리
        const auto cur_mb_seq = ir_mb_.latest_seq();
        if (cur_mb_seq == ir_mb_seq_seen_) {
            // 새 프레임 없음
            continue;
        }

        if (auto hopt = ir_mb_.exchange(nullptr)) {
            std::shared_ptr<IRFrameHandle> h = *hopt;
            if (h) {
                // 메일박스 seq 갱신
                ir_mb_seq_seen_ = cur_mb_seq;

                // 프레임 고유 seq 저장 (이걸 이벤트에 실어보냄)
                last_frame_seq_ = h->seq;

                on_frame(*h);   // 타임라인 로깅은 on_frame 내부
                h->release();   // 파생형 release() 허용(기본 no-op)
            }
        }
    }
    cleanup();
}

void IR_TrackThread::wait_until_ready() {
    std::unique_lock<std::mutex> lk(m_);
    cv_.wait(lk, [&]{
        return !running_.load()
        || click_mb_.latest_seq() > click_mb_seq_seen_
        || ir_mb_.latest_seq()    > ir_mb_seq_seen_;
    });
    // 탈출: 종료 or 새 데이터 존재
}

void IR_TrackThread::handle_click(const UserCmd& cmd) {
    target_box_         = cmd.box;
    new_target_         = true;
    click_mb_seq_seen_  = cmd.seq;
    fail_streak_        = 0;
    reselect_notified_  = false;

    // 콘솔 로그
    LOGI(TAG, "click: (%.1f, %.1f, %.1f, %.1f) seq=%u",
         cmd.box.x, cmd.box.y, cmd.box.width, cmd.box.height, cmd.seq);

    // TL 포맷으로 클릭 이벤트도 남겨둔다 (즉시 이벤트)
    const std::uint64_t t_us = now_us_steady();
    std::ostringstream oss;
    oss << "CLICK"
        << ",x=" << cmd.box.x
        << ",y=" << cmd.box.y
        << ",w=" << cmd.box.width
        << ",h=" << cmd.box.height;

    CSV_LOG_TL("IR.Track",
               static_cast<unsigned long>(cmd.seq),
               t_us, t_us, t_us, t_us,
               0,
               oss.str());
}

void IR_TrackThread::on_frame(IRFrameHandle& h) {
    // ── 타임라인 측정 시작 ──
    std::uint64_t t0_us = now_us_steady();
    std::uint64_t t1_us = 0;
    std::uint64_t t2_us = 0;
    std::uint64_t t3_us = 0;

    double pre_us = 0.0;
    double upd_us = 0.0;

    std::string note;

    // 1) 프리프로세싱
    cv::Mat pf32;
    {
        std::uint64_t t_pre0 = now_us_steady();
        preproc_.run(*h.p, pf32);
        std::uint64_t t_pre1 = now_us_steady();
        pre_us = static_cast<double>(t_pre1 - t_pre0);
        t1_us  = t_pre1;
    }

    // 2) 새 타깃 초기화 시도
    if (new_target_) {
        std::uint64_t t_upd0 = now_us_steady();
        bool init_ok = try_init(pf32, target_box_);
        std::uint64_t t_upd1 = now_us_steady();
        upd_us = static_cast<double>(t_upd1 - t_upd0);
        t2_us  = t_upd1;

        if (init_ok) {
            tracking_valid_    = true;
            new_target_        = false;
            fail_streak_       = 0;
            reselect_notified_ = false;

            emit_init(target_box_, h.ts);

            LOGI(TAG, "init OK (seq=%u, pre=%.2f us)", h.seq, pre_us);
            std::ostringstream oss;
            oss << "INIT_OK"
                << ",pre_us=" << pre_us
                << ",upd_us=" << upd_us
                << ",x=" << target_box_.x
                << ",y=" << target_box_.y
                << ",w=" << target_box_.width
                << ",h=" << target_box_.height;
            note = oss.str();
        } else {
            tracking_valid_ = false;
            new_target_     = false;
            fail_streak_++;

            LOGW(TAG, "init FAIL #%d (pre=%.2f us)", fail_streak_, pre_us);
            emit_lost(target_box_, h.ts);

            bool need_reselect = false;
            if (fail_streak_ >= cfg_.user_req_threshold && !reselect_notified_) {
                reselect_notified_ = true;
                emit_need_reselect();
                need_reselect = true;
            }

            std::ostringstream oss;
            oss << "INIT_FAIL"
                << ",pre_us=" << pre_us
                << ",upd_us=" << upd_us
                << ",streak=" << fail_streak_
                << ",need_reselect=" << (need_reselect ? 1 : 0);
            note = oss.str();
        }

        t3_us = now_us_steady();
        CSV_LOG_TL("IR.Track",
                   static_cast<unsigned long>(h.seq),
                   t0_us, t1_us, t2_us, t3_us,
                   0,
                   note);
        return;
    }

    // 3) 이미 타깃 상실 상태: 누적만
    if (!tracking_valid_) {
        fail_streak_++;

        LOGW(TAG, "lost (streak=%d, pre=%.2f us)", fail_streak_, pre_us);

        if (fail_streak_ == 1) {
            emit_lost(target_box_, h.ts); // 첫 상실만 Lost 이벤트
        }

        bool need_reselect = false;
        if (fail_streak_ >= cfg_.user_req_threshold && !reselect_notified_) {
            reselect_notified_ = true;
            emit_need_reselect();
            need_reselect = true;
        }

        t2_us = t1_us; // 추가 업데이트는 없음
        t3_us = now_us_steady();

        std::ostringstream oss;
        oss << "LOST"
            << ",pre_us=" << pre_us
            << ",streak=" << fail_streak_
            << ",need_reselect=" << (need_reselect ? 1 : 0);
        note = oss.str();

        CSV_LOG_TL("IR.Track",
                   static_cast<unsigned long>(h.seq),
                   t0_us, t1_us, t2_us, t3_us,
                   0,
                   note);
        return;
    }

    // 4) 유효 상태: 업데이트
    cv::Rect2f out; float score = 0.f;
    bool ok = false;
    {
        std::uint64_t t_upd0 = now_us_steady();
        ok = try_update(pf32, out, score);
        std::uint64_t t_upd1 = now_us_steady();
        upd_us = static_cast<double>(t_upd1 - t_upd0);
        t2_us  = t_upd1;
    }

    if (ok) {
        target_box_        = out;
        fail_streak_       = 0;
        reselect_notified_ = false;
        emit_track(out, score, h.ts);

        LOGD(TAG, "track OK (seq=%u, pre=%.2f us, upd=%.2f us, score=%.3f, box=%.1f,%.1f,%.1f,%.1f)",
             h.seq, pre_us, upd_us, score, out.x, out.y, out.width, out.height);

        std::ostringstream oss;
        oss << "TRACK_OK"
            << ",pre_us=" << pre_us
            << ",upd_us=" << upd_us
            << ",score="  << score
            << ",x=" << out.x
            << ",y=" << out.y
            << ",w=" << out.width
            << ",h=" << out.height;
        note = oss.str();
    } else {
        tracking_valid_ = false;
        fail_streak_++;

        LOGW(TAG, "track LOST (seq=%u, streak=%d, pre=%.2f us, upd=%.2f us)",
             h.seq, fail_streak_, pre_us, upd_us);

        emit_lost(target_box_, h.ts);

        bool need_reselect = false;
        if (fail_streak_ >= cfg_.user_req_threshold && !reselect_notified_) {
            reselect_notified_ = true;
            emit_need_reselect();
            need_reselect = true;
        }

        std::ostringstream oss;
        oss << "TRACK_LOST"
            << ",pre_us=" << pre_us
            << ",upd_us=" << upd_us
            << ",streak=" << fail_streak_
            << ",need_reselect=" << (need_reselect ? 1 : 0);
        note = oss.str();
    }

    t3_us = now_us_steady();

    // 이 프레임에 대해 딱 1줄 타임라인 기록
    CSV_LOG_TL("IR.Track",
               static_cast<unsigned long>(h.seq),
               t0_us, t1_us, t2_us, t3_us,
               0,
               note);
}

bool IR_TrackThread::try_init(const cv::Mat& pf, const cv::Rect2f& box) {
    return tracker_.init(pf, box);
}
bool IR_TrackThread::try_update(const cv::Mat& pf, cv::Rect2f& out_box, float& score) {
    return tracker_.update(pf, out_box, score);
}

// ==== EventBus 발행 ====
void IR_TrackThread::emit_init(const cv::Rect2f& b, uint64_t ts) {
    bus_.push(Event{ EventType::Init,  InitEvent{ b, ts, ir_mb_seq_seen_ } }, Topic::Tracking);
}
void IR_TrackThread::emit_track(const cv::Rect2f& b, float score, uint64_t ts) {
    bus_.push(Event{ EventType::Track, TrackEvent{ b, score, ts, ir_mb_seq_seen_ } }, Topic::Tracking);
}
void IR_TrackThread::emit_lost(const cv::Rect2f& last, uint64_t ts) {
    bus_.push(Event{ EventType::Lost,  LostEvent{ last, ts, ir_mb_seq_seen_ } }, Topic::Tracking);
}
void IR_TrackThread::emit_need_reselect() {
    bus_.push(Event{ EventType::NeedReselect, NeedReselectEvent{} }, Topic::Tracking);
}

void IR_TrackThread::cleanup() {
    // 필요 시 tracker 리셋/버스 정리
    // tracker_.reset();
    // bus_.unsubscribe(...);
}

} // namespace flir

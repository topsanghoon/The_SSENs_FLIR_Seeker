// IR_TrackThread.cpp  (CSV unified / CsvLoggerIR 제거)
#include "threads_includes/IR_TrackThread.hpp"
#include <condition_variable>
#include <mutex>

#include "util/csv_sink.hpp"   // ✅ 단일 CSV 싱크
#include "util/telemetry.hpp"
#include "phase_gate.hpp"
/*
종말 유도가 아니면 tracking 스레드에 데이터가 들어오지 않음으로 깨어날 일 없음.
중기 유도가 끝나면 중기 ArUco 스레드가 종료되면서 전역 변수를 바꿀것이고, 바꾸면 이후 캡쳐 과정에서 IR 프레임을
트래킹 스레드로 전송하게 되고, 그때부터 자연스럽게 동작
*/

namespace flir {

static constexpr const char* TAG = "IR.Track";

// wake와 관리를 위한 전역 변수(프로세스 내 단일 트래커 가정)
static std::mutex              g_m;
static std::condition_variable g_cv;

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
    CSV_LOG_SIMPLE("IR.Track", "THREAD_START", 0, 0,0,0,0, "");
    th_ = std::thread(&IR_TrackThread::run, this);
}
void IR_TrackThread::stop() {
    if (!running_.load()) return;
    running_.store(false);
    g_cv.notify_all(); // 대기 중이면 깨워서 종료 경로로
}
void IR_TrackThread::join() {
    if (th_.joinable()) {
        th_.join();
        CSV_LOG_SIMPLE("IR.Track", "THREAD_STOP", 0, 0,0,0,0, "");
    }
}

/*
프레임/클릭이 도착하면 메일박스에 넣고 깨움.
프레임이 와야 한 스텝이 진행된다.
*/
void IR_TrackThread::onFrameArrived(std::shared_ptr<IRFrameHandle> h) {
    ir_mb_.push(std::move(h));
    g_cv.notify_one();
}
void IR_TrackThread::onClickArrived(const UserCmd& cmd) {
    click_mb_.push(cmd);
    g_cv.notify_one();
}

// 메인 루프
void IR_TrackThread::run() {
    while (running_.load()) {
        if (!flir::ir_enabled()) { std::unique_lock<std::mutex> lk(g_m); g_cv.wait_for(lk, std::chrono::milliseconds(5)); continue; }

        wait_until_ready();
        if (!running_.load()) break;

        if (click_mb_.has_new(click_seq_seen_)) {
            if (auto cmd = click_mb_.exchange(nullptr)) {
                handle_click(*cmd);
            }
        }
        if (!ir_mb_.has_new(frame_seq_seen_)) continue;

        if (auto hopt = ir_mb_.exchange(nullptr)) {
            std::shared_ptr<IRFrameHandle> h = *hopt; // tx와 공유하므로 shared_ptr
            if (h) {
                double loop_ms = 0.0;
                CSV_LOG_SIMPLE("IR.Track", "LOOP_BEGIN", h->seq, 0,0,0,0, "");
                {
                    ScopedTimerMs t(loop_ms);
                    on_frame(*h);   // 참조로 넘겨 파생형 가상함수 유지
                }
                CSV_LOG_SIMPLE("IR.Track", "LOOP_END",   h->seq, loop_ms, 0,0,0, "");
                h->release();   // 파생형 release() 허용(기본 no-op)
            }
        }
    }
    cleanup();
}

void IR_TrackThread::wait_until_ready() {
    std::unique_lock<std::mutex> lk(g_m);
    g_cv.wait(lk, [&]{
        return !running_.load() ||
               click_mb_.latest_seq() > click_seq_seen_ ||
               ir_mb_.latest_seq()    > frame_seq_seen_;
    });
    // 탈출: 종료 or 새 데이터 존재
}

void IR_TrackThread::handle_click(const UserCmd& cmd) {
    target_box_         = cmd.box;
    new_target_         = true;
    click_seq_seen_     = cmd.seq;
    fail_streak_        = 0;
    reselect_notified_  = false;

    // 콘솔 로그 + CSV(좌표/크기 4개를 v1..v4에 기록)
    LOGI(TAG, "click: (%.1f, %.1f, %.1f, %.1f) seq=%u",
         cmd.box.x, cmd.box.y, cmd.box.width, cmd.box.height, cmd.seq);
    CSV_LOG_SIMPLE("IR.Track", "CLICK", cmd.seq,
                   cmd.box.x, cmd.box.y, cmd.box.width, cmd.box.height, "");
}

void IR_TrackThread::on_frame(IRFrameHandle& h) {
    frame_seq_seen_ = h.seq;

    // 1) 프리프로세싱
    cv::Mat pf32;
    double pre_ms = 0.0;
    { flir::ScopedTimerMs t(pre_ms); preproc_.run(*h.p, pf32); }
    CSV_LOG_SIMPLE("IR.Track", "PRE_MS", h.seq, pre_ms, 0,0,0, "");

    // 2) 새 타깃 초기화 시도
    if (new_target_) {
        if (try_init(pf32, target_box_)) {
            tracking_valid_    = true;
            new_target_        = false;
            fail_streak_       = 0;
            reselect_notified_ = false;

            emit_init(target_box_, h.ts);

            LOGI(TAG, "init OK (seq=%u, pre=%.2f ms)", h.seq, pre_ms);
            CSV_LOG_SIMPLE("IR.Track", "INIT_OK", h.seq, pre_ms, 0,0,0, "");
        } else {
            tracking_valid_ = false;
            new_target_     = false;
            fail_streak_++;

            LOGW(TAG, "init FAIL #%d (pre=%.2f ms)", fail_streak_, pre_ms);
            CSV_LOG_SIMPLE("IR.Track", "INIT_FAIL", h.seq, pre_ms, fail_streak_, 0,0, "");

            if (fail_streak_ == 1) emit_lost(target_box_, h.ts);
            if (fail_streak_ >= cfg_.user_req_threshold && !reselect_notified_) {
                reselect_notified_ = true;
                emit_need_reselect();
                CSV_LOG_SIMPLE("IR.Track", "NEED_RESELECT", h.seq, fail_streak_, 0,0,0, "");
            }
        }
        return;
    }

    // 3) 이미 타깃 상실 상태: 누적만
    if (!tracking_valid_) {
        fail_streak_++;
        LOGW(TAG, "lost (streak=%d, pre=%.2f ms)", fail_streak_, pre_ms);
        CSV_LOG_SIMPLE("IR.Track", "LOST", h.seq, pre_ms, fail_streak_, 0,0, "");

        if (fail_streak_ == 1) {
            emit_lost(target_box_, h.ts); // 첫 상실만 Lost 이벤트
        }
        if (fail_streak_ >= cfg_.user_req_threshold && !reselect_notified_) {
            reselect_notified_ = true;
            emit_need_reselect();
            CSV_LOG_SIMPLE("IR.Track", "NEED_RESELECT", h.seq, fail_streak_, 0,0,0, "");
        }
        return;
    }

    // 4) 유효 상태: 업데이트
    cv::Rect2f out; float score = 0.f;
    double upd_ms = 0.0;
    bool ok = false;
    { flir::ScopedTimerMs t(upd_ms); ok = try_update(pf32, out, score); }
    CSV_LOG_SIMPLE("IR.Track", "UPD_MS", h.seq, upd_ms, 0,0,0, "");

    if (ok) {
        target_box_        = out;
        fail_streak_       = 0;
        reselect_notified_ = false;
        emit_track(out, score, h.ts);

        LOGD(TAG, "track OK (seq=%u, pre=%.2f ms, upd=%.2f ms, score=%.3f, box=%.1f,%.1f,%.1f,%.1f)",
             h.seq, pre_ms, upd_ms, score, out.x, out.y, out.width, out.height);
        // v1=score, v2=upd_ms, v3=pre_ms (후분석 가독성)
        CSV_LOG_SIMPLE("IR.Track", "TRACK_OK", h.seq, score, upd_ms, pre_ms, 0,
                       "x=" + std::to_string(out.x) +
                       ",y=" + std::to_string(out.y) +
                       ",w=" + std::to_string(out.width) +
                       ",h=" + std::to_string(out.height));
    } else {
        tracking_valid_ = false;
        fail_streak_++;

        LOGW(TAG, "track LOST (seq=%u, streak=%d, pre=%.2f ms, upd=%.2f ms)",
             h.seq, fail_streak_, pre_ms, upd_ms);
        // v1=pre_ms, v2=upd_ms, v3=streak
        CSV_LOG_SIMPLE("IR.Track", "TRACK_LOST", h.seq, pre_ms, upd_ms, fail_streak_, 0, "");

        emit_lost(target_box_, h.ts);

        if (fail_streak_ >= cfg_.user_req_threshold && !reselect_notified_) {
            reselect_notified_ = true;
            emit_need_reselect();
            CSV_LOG_SIMPLE("IR.Track", "NEED_RESELECT", h.seq, fail_streak_, 0,0,0, "");
        }
    }
}

bool IR_TrackThread::try_init(const cv::Mat& pf, const cv::Rect2f& box) {
    return tracker_.init(pf, box);
}
bool IR_TrackThread::try_update(const cv::Mat& pf, cv::Rect2f& out_box, float& score) {
    return tracker_.update(pf, out_box, score);
}

// ==== EventBus 발행 ====
void IR_TrackThread::emit_init(const cv::Rect2f& b, uint64_t ts) {
    bus_.push(Event{ EventType::Init,  InitEvent{ b, ts, frame_seq_seen_ } }, Topic::Tracking);
}
void IR_TrackThread::emit_track(const cv::Rect2f& b, float score, uint64_t ts) {
    bus_.push(Event{ EventType::Track, TrackEvent{ b, score, ts, frame_seq_seen_ } }, Topic::Tracking);
}
void IR_TrackThread::emit_lost(const cv::Rect2f& last, uint64_t ts) {
    bus_.push(Event{ EventType::Lost,  LostEvent{ last, ts, frame_seq_seen_ } }, Topic::Tracking);
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

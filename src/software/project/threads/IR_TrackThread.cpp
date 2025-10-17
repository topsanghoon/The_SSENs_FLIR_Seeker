#include "threads_includes/IR_TrackThread.hpp"
#include <condition_variable>
#include <mutex>

namespace flir {

// 여기선 예시로 전역 CV를 사용했지만, 실제론 ThreadManager가 WakeHandle로 관리해도 됨.
static std::mutex              g_m;
static std::condition_variable g_cv;

IR_TrackThread::IR_TrackThread(SpscMailbox<IRFrameHandle>& ir_mb,
                               SpscMailbox<UserCmd>&     click_mb,
                               ITrackerStrategy&         tracker,
                               IPreprocessor&            preproc,
                               IReinitHintPolicy&        reinit,
                               IEventBus&                bus,
                               CsvLogger                 logger,
                               IRTrackConfig             cfg)
: ir_mb_(ir_mb)
, click_mb_(click_mb)
, tracker_(tracker)
, preproc_(preproc)
, reinit_(reinit)
, bus_(bus)
, log_(std::move(logger))
, cfg_(cfg)
{}

void IR_TrackThread::start() {
    running_.store(true);
    th_ = std::thread(&IR_TrackThread::run, this);
}

void IR_TrackThread::stop() {
    running_.store(false);
    g_cv.notify_all(); // 잠자고 있으면 깨워서 종료 경로로 빠지게
}

void IR_TrackThread::join() {
    if (th_.joinable()) th_.join();
}

void IR_TrackThread::onClickArrived(const UserCmd& cmd) {
    click_mb_.push(cmd);
    g_cv.notify_one(); // run()을 깨우는 신호
}

void IR_TrackThread::onFrameArrived(IRFrameHandle h) {
    ir_mb_.push(h);
    g_cv.notify_one(); // run()을 깨우는 신호
}

void IR_TrackThread::run() {
    // 초기화(필요 시)
    // initTrackerStuff();

    while (running_.load()) {
        wait_until_ready();
        if (!running_.load()) break;

        // CLICK 우선 흡수(초기화는 다음 프레임에서)
        if (click_mb_.has_new(click_seq_seen_)) {
            if (auto cmd = click_mb_.exchange(nullptr)) {
                handle_click(*cmd);
            }
        }

        // 프레임이 없으면 다음 대기
        if (!ir_mb_.has_new(frame_seq_seen_)) continue;

        // 프레임 소비 및 처리
        if (auto h = ir_mb_.exchange(nullptr)) {
            IRFrameHandle fh = *h; // (필요 시 이동)
            on_frame(fh);        // 전처리 → init/update → 이벤트 발행/로깅
            fh.release();        // 사용 후 반환
        }
    }

    // cleanup();
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
    // 목적: 클릭 정보를 상태에 반영(다음 프레임에서 init 시도)
    // target_box_      = cmd.box;
    // new_target_      = true;
    // click_seq_seen_  = cmd.seq;
    // log_.click(cmd.seq, cmd.box);
}

void IR_TrackThread::on_frame(IRFrameHandle& h) {
    frame_seq_seen_ = h.seq;

    cv::Mat pf32;
    preproc_.run(*h.p, pf32);
    double ms=0; { flir::ScopedTimer t(ms); preproc_.run(*h.p, pf32); }

    if (new_target_) {
        if (try_init(pf32, target_box_)) {
            tracking_valid_ = true; new_target_ = false; fail_streak_ = 0;
            emit_init(target_box_, h.ts);
            log_.init_ok(h.seq, ms);
        } else {
            tracking_valid_ = false; new_target_ = false; fail_streak_++;
            log_.init_fail(fail_streak_, ms);
        }
        return;
    }

    if (tracking_valid_) {
        cv::Rect2f out; float score = 0.f;
        if (try_update(pf32, out, score)) {
            target_box_ = out; fail_streak_ = 0;
            emit_track(out, score, h.ts);
            log_.track_ok(h.seq, score, ms);
        } else {
            tracking_valid_ = false; fail_streak_++;
            emit_lost(target_box_, h.ts);
            log_.track_lost(fail_streak_, ms);
            if (fail_streak_ == cfg_.reinit_threshold) {
                target_box_ = reinit_.suggest(target_box_);
                new_target_ = true;
            }
            if (fail_streak_ >= cfg_.user_req_threshold) {
                emit_need_reselect();
            }
        }
    }
}


bool IR_TrackThread::try_init(const cv::Mat& pf, const cv::Rect2f& box) {
    // return tracker_.init(pf, box);
    (void)pf; (void)box; return false;
}

bool IR_TrackThread::try_update(const cv::Mat& pf, cv::Rect2f& out_box, float& score) {
    // return tracker_.update(pf, out_box, score);
    (void)pf; (void)out_box; (void)score; return false;
}

void IR_TrackThread::emit_init(const cv::Rect2f& b, uint64_t ts) {
    // bus_.push(Event{ EventType::Init, InitEvent{b, ts, frame_seq_seen_} }, /*topic=*/0);
    (void)b; (void)ts;
}

void IR_TrackThread::emit_track(const cv::Rect2f& b, float score, uint64_t ts) {
    // bus_.push(Event{ EventType::Track, TrackEvent{b, score, ts, frame_seq_seen_} }, /*topic=*/0);
    (void)b; (void)score; (void)ts;
}

void IR_TrackThread::emit_lost(const cv::Rect2f& last, uint64_t ts) {
        // bus_.push(makeTrackEv(b, score, ts, frame_seq_seen_), Topic::Tracking);
    (void)last; (void)ts;
}

void IR_TrackThread::emit_need_reselect() {
    // bus_.push(Event{ EventType::NeedReselect, NeedReselectEvent{} }, /*topic=*/0);
}

} // namespace flir

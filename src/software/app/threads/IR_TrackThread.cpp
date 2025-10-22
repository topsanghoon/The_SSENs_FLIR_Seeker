#include "threads_includes/IR_TrackThread.hpp"
#include <condition_variable>
#include <mutex>

/*
종말 유도가 아니면 tracking 스레드에 데이터가 들어오지 않음으로 깨어날 일 없음.
중기 유도와 종말 유도를 구분할 수 있는 전역 변수가 존재하는데

중기 유도가 끝나면 중기 ArUco 스레드가 종료되면서 전역 변수를 바꿀것이고, 바꾸면 이후 캡쳐 과정에서 IR 프레임을
트래킹 스레드로 전송하게 되고, 그때부터 자연스럽게 동작

따라서 트래킹 스레드는 충실하게 데이터가 들어왔을 때 동작하는 구현만 있으면 되는 것
*/

namespace flir {

//wake와 관리를 위한 전역 변수.
static std::mutex              g_m;
static std::condition_variable g_cv;

IR_TrackThread::IR_TrackThread(SpscMailbox<std::shared_ptr<IRFrameHandle>>& ir_mb,
                               SpscMailbox<UserCmd>&     click_mb,
                               ITrackerStrategy&         tracker,
                               IPreprocessor&            preproc,
                               IEventBus&                bus,
                               CsvLoggerIR&              logger,
                               IRTrackConfig             cfg)
: ir_mb_(ir_mb)
, click_mb_(click_mb)
, tracker_(tracker)
, preproc_(preproc)
, bus_(bus)
, log_(logger)
, cfg_(cfg)
{}


// start, stop, join은 스레드가 동작하는 기본적이 구현이 되어있는 부분
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


/*
프레임이 도착하거나, 클릭 이벤트가 발생했을 때, 필요한 data를 전달하고 스레드를 깨움
클릭 이벤트가 전달되면 새로운 대상을 트래킹하기 위한 준비를 하고, 프레임이 아직 도달하지 않았을 경우, 다시 잠듦
결국 프레임이 도착해야만 한번의 동작이 실시되는 것
*/
void IR_TrackThread::onFrameArrived(std::shared_ptr<IRFrameHandle> h) {
    ir_mb_.push(std::move(h));
    g_cv.notify_one();
}

void IR_TrackThread::onClickArrived(const UserCmd& cmd) {
    click_mb_.push(cmd);
    g_cv.notify_one(); // 대기 중이면 깨움
}




// 메인 동작 흐름
void IR_TrackThread::run() {
    while (running_.load()) {   //스레드가 동작 중이라면(스레드 종료를 위해 있는 내용)
        wait_until_ready();     //준비 될때까지 기다림
        if (!running_.load()) break;        //중간에 일어나봤는데 종료되었을 수 있음으로 안전하게 확인

        if (click_mb_.has_new(click_seq_seen_)) {       //새로운 클릭이 도착했을 때 동작. 이전 클릭에 대한 seq id를 통해 구분, seq는 Net_RxThread에서 증가시켜줌 
            if (auto cmd = click_mb_.exchange(nullptr)) {       //클릭 정보 뽑아옴
                handle_click(*cmd);                     //클릭 동작 수행
            }
        }
        if (!ir_mb_.has_new(frame_seq_seen_)) continue; //클릭 처리했는데 프레임이 없으면 -> 다시

        if (auto hopt = ir_mb_.exchange(nullptr)) {     //프레임 정보 뽑아옴
            std::shared_ptr<IRFrameHandle> h = *hopt;   //전송과 tracking 둘다 소비하는 데이터임으로 shared_ptr 사용
            if (h) {
                on_frame(*h);     // 참조로 넘김 → 가상함수/동적타입 유지
                h->release();     // 파생형의 release() 호출 가능
            }
        }
    }
    cleanup();
}

void IR_TrackThread::wait_until_ready() {
    std::unique_lock<std::mutex> lk(g_m);
    g_cv.wait(lk, [&]{
        return !running_.load() ||
               click_mb_.latest_seq() > click_seq_seen_ ||  //새로운 데이터가 들어오면
               ir_mb_.latest_seq()    > frame_seq_seen_;
    });
    // 탈출: 종료 or 새 데이터 존재
}

void IR_TrackThread::handle_click(const UserCmd& cmd) {
    target_box_      = cmd.box;
    new_target_      = true;
    click_seq_seen_  = cmd.seq;
    fail_streak_     = 0;
    reselect_notified_ = false;
    log_.click(cmd.seq, cmd.box);
}

void IR_TrackThread::on_frame(IRFrameHandle& h) {
    frame_seq_seen_ = h.seq;

    cv::Mat pf32;
    double ms = 0;
    { ScopedTimer t(ms); preproc_.run(*h.p, pf32); }

    // 1) 새 타깃 초기화 시도
    if (new_target_) {
        if (try_init(pf32, target_box_)) {
            tracking_valid_    = true;
            new_target_        = false;
            fail_streak_       = 0;
            reselect_notified_ = false;
            emit_init(target_box_, h.ts);
            log_.init_ok(h.seq, ms);
        } else {
            tracking_valid_ = false;
            new_target_     = false;
            fail_streak_++;
            log_.init_fail(fail_streak_, ms);

            if (fail_streak_ == 1) emit_lost(target_box_, h.ts);
            if (fail_streak_ >= cfg_.user_req_threshold && !reselect_notified_) {
                reselect_notified_ = true;
                emit_need_reselect();
            }
        }
        return;
    }

    // 2) 이미 타깃을 잃은 상태라면: 프레임마다 누적만(재초기화 없음)
    if (!tracking_valid_) {
        fail_streak_++;
        log_.track_lost(fail_streak_, ms);

        if (fail_streak_ == 1) {
            // 첫 잃어버림만 Lost 이벤트 발행 (디바운스)
            emit_lost(target_box_, h.ts);
        }
        if (fail_streak_ >= cfg_.user_req_threshold && !reselect_notified_) {
            reselect_notified_ = true;
            emit_need_reselect();
        }
        return;
    }

    // 3) 유효한 상태: 업데이트 시도
    cv::Rect2f out; float score = 0.f;
    if (try_update(pf32, out, score)) {
        target_box_        = out;
        fail_streak_       = 0;
        reselect_notified_ = false;
        emit_track(out, score, h.ts);
        log_.track_ok(h.seq, score, ms);
    } else {
        tracking_valid_ = false;
        fail_streak_++;
        log_.track_lost(fail_streak_, ms);

        // Lost는 첫 번만
        emit_lost(target_box_, h.ts);

        if (fail_streak_ >= cfg_.user_req_threshold && !reselect_notified_) {
            reselect_notified_ = true;
            emit_need_reselect();
        }
    }
}



bool IR_TrackThread::try_init(const cv::Mat& pf, const cv::Rect2f& box) {
    return tracker_.init(pf, box);
}

bool IR_TrackThread::try_update(const cv::Mat& pf, cv::Rect2f& out_box, float& score) {
    return tracker_.update(pf, out_box, score);
}

void IR_TrackThread::emit_init(const cv::Rect2f& b, uint64_t ts) {
    bus_.push(Event{
        EventType::Init,
        InitEvent{ b, ts, frame_seq_seen_ }
    }, Topic::Tracking);
}

void IR_TrackThread::emit_track(const cv::Rect2f& b, float score, uint64_t ts) {
    bus_.push(Event{
        EventType::Track,
        TrackEvent{ b, score, ts, frame_seq_seen_ }
    }, Topic::Tracking);
}

void IR_TrackThread::emit_lost(const cv::Rect2f& last, uint64_t ts) {
    bus_.push(Event{
        EventType::Lost,
        LostEvent{ last, ts, frame_seq_seen_ }
    }, Topic::Tracking);
}

void IR_TrackThread::emit_need_reselect() {
    bus_.push(Event{
        EventType::NeedReselect,
        NeedReselectEvent{}
    }, Topic::Tracking);
}


void IR_TrackThread::cleanup() { //todo
    //tracker_.reset();     // OpenCV 트래커 해제
    //bus_.unsubscribe(...);
}

} // namespace flir

// src/software/app/tests/IR_TrackThread_sanity.cpp
#include <atomic>
#include <thread>
#include <vector>
#include <iostream>
#include <chrono>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>

#include "threads_includes/IR_TrackThread.hpp"     // SUT
#include "ipc/ipc_types.hpp"
#include "ipc/event_bus.hpp"
#include "ipc/mailbox.hpp"
#include "components/includes/IR_Frame.hpp"
#include "components/includes/CsvLoggerIR.hpp"     // 실제 CSV 로거 사용

using namespace std::chrono_literals;

namespace flir {

// ---------- 가짜 컴포넌트들 ----------
struct FakePreproc : IPreprocessor {
    void run(const IRFrame16& in16, cv::Mat& out32f) override {
        out32f.create(in16.height, in16.width, CV_32F);
        out32f.setTo(0.5f);
    }
};

struct FakeTracker : ITrackerStrategy {
    bool init_ok_ = true;
    int  updates_ok_before_fail_ = 100; // N 프레임 후 실패 유도
    int  call_count_ = 0;
    cv::Rect2f cur_{};

    bool init(const cv::Mat&, const cv::Rect2f& box) override {
        call_count_ = 0; cur_ = box; return init_ok_;
    }
    bool update(const cv::Mat&, cv::Rect2f& out_box, float& score) override {
        if (call_count_ >= updates_ok_before_fail_) { score = 0.f; return false; }
        cur_.x += 2.f; cur_.y += 1.f;
        out_box = cur_;
        score = 1.0f;
        ++call_count_;
        return true;
    }
};

struct FakeReinitHint : IReinitHintPolicy {
    cv::Rect2f suggest(const cv::Rect2f& last_box) override {
        auto r = last_box; r.x += 5.f; r.y += 3.f; return r;
    }
};

struct FakeBus : IEventBus {
    std::vector<Event> evs;
    void subscribe(Topic, SpscMailbox<Event>*, WakeHandle*) override {}
    void unsubscribe(SpscMailbox<Event>*) override {}
    void push(const Event& e, Topic) override { evs.push_back(e); }
};

// ---------- 간이 프레임 핸들 ----------
struct TestFrame {
    std::vector<uint16_t> buf; IRFrame16 fr{};
    TestFrame(int w,int h){
        buf.resize((size_t)w*h);
        fr.data=buf.data(); fr.width=w; fr.height=h; fr.step=w*2;
    }
};
struct TestHandle : IRFrameHandle {
    std::shared_ptr<TestFrame> owner;
    void retain() {}              // override 빼기
    void release() { owner.reset(); }
};

void run_sanity() {
    SpscMailbox<IRFrameHandle> mb_frame(64);
    SpscMailbox<UserCmd>       mb_click(16);
    FakeTracker     tracker;
    FakePreproc     preproc;
    FakeReinitHint  rehint;
    FakeBus         bus;
    CsvLoggerIR     logger("/tmp/ir_track_test.csv");   // 실제 로거

    IRTrackConfig cfg; cfg.reinit_threshold=3; cfg.user_req_threshold=9;

    IR_TrackThread th(mb_frame, mb_click, tracker, preproc, rehint, bus, logger, cfg);
    th.start();

    // 1) 클릭으로 ROI 지정
    UserCmd cmd{}; cmd.type = CmdType::CLICK; cmd.box = cv::Rect2f(20,30,40,50); cmd.seq = 1;
    th.onClickArrived(cmd);

    // 2) 정상 추적 프레임 5개
    for (uint32_t i=1; i<=5; ++i) {
        auto tf = std::make_shared<TestFrame>(160,120);
        TestHandle h; h.owner=tf; h.p=&tf->fr; h.seq=i; h.ts=i*10;
        th.onFrameArrived(h);
        std::this_thread::sleep_for(5ms);
    }

    // 3) 실패 유도: 2회만 OK → 이후 LOST 누적
    tracker.updates_ok_before_fail_ = 2;
    for (uint32_t i=6; i<=12; ++i) {
        auto tf = std::make_shared<TestFrame>(160,120);
        TestHandle h; h.owner=tf; h.p=&tf->fr; h.seq=i; h.ts=i*10;
        th.onFrameArrived(h);
        std::this_thread::sleep_for(5ms);
    }

    th.stop();
    th.join();

    size_t init_n=0, track_n=0, lost_n=0, need_n=0;
    for (auto& e : bus.evs) {
        switch (e.type) {
            case EventType::Init: init_n++; break;
            case EventType::Track: track_n++; break;
            case EventType::Lost: lost_n++; break;
            case EventType::NeedReselect: need_n++; break;
            default: break;
        }
    }
    std::cout << "[TEST] Init="<<init_n<<" Track="<<track_n
              << " Lost="<<lost_n<<" NeedReselect="<<need_n<<"\n";
}

} // namespace flir

int main(){ flir::run_sanity(); return 0; }

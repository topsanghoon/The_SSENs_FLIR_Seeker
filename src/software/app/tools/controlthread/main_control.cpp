#include <iostream>
#include <thread>
#include <chrono>
#include <variant>
#include <cstdint>
#include <algorithm>
#include <vector>
#include <atomic>
#include <mutex>

#include "threads_includes/ControlThread.hpp"
#include "components/includes/ControlDTO.hpp"
#include "components/includes/TargetFusion.hpp"
#include "components/includes/IController.hpp"   // ControllerLR 포함
#include "components/includes/ActuatorPort.hpp"
#include "components/includes/CsvLoggerCtrl.hpp"
#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/event_bus.hpp"

namespace flir {

static constexpr int   FRAME_W      = 160;
static constexpr float DEADZONE_PX  = 4.0f;

// UART 대체 포트
class PrintActuator final : public IActuatorPort {
public:
    void write_nonblock(const CtrlCmd& cmd) override {
        if (quiesce_.load()) return;
        const char* dir = (cmd.mode < 0 ? "LEFT"
                           : cmd.mode > 0 ? "RIGHT" : "CENTER");
        std::cout << "[UART-FAKE] DIR=" << dir << " (err=" << cmd.p1 << ")\n";
    }
    void set_quiesce(bool q) override {
        quiesce_.store(q);
        std::cout << "[UART-FAKE] set_quiesce=" << (q?"true":"false") << "\n";
    }
    void stop_io() override {
        std::cout << "[UART-FAKE] stop_io\n";
    }
private:
    std::atomic<bool> quiesce_{false};
};

// EventBus(Mock)
class MockEventBus : public IEventBus {
public:
    void subscribe(Topic t, SpscMailbox<Event>* inbox, WakeHandle* wh) override {
        std::lock_guard<std::mutex> lk(mu_);
        subs_.push_back(Sub{t, inbox, wh});
    }
    void unsubscribe(SpscMailbox<Event>* inbox) override {
        std::lock_guard<std::mutex> lk(mu_);
        subs_.erase(std::remove_if(subs_.begin(), subs_.end(),
                     [&](const Sub& s){ return s.inbox == inbox; }), subs_.end());
    }
    void push(const Event& ev, Topic t) override {
        std::lock_guard<std::mutex> lk(mu_);
        if (t == Topic::Control && std::holds_alternative<MetaCtrlEvent>(ev.payload)) {
            std::cout << "[BUS][Control] MetaCtrl received\n";
        }
        for (auto& s : subs_) {
            if (s.topic == t) {
                s.inbox->push(ev);
                if (s.wh) s.wh->signal();
            }
        }
    }
private:
    struct Sub {
        Topic               topic;
        SpscMailbox<Event>* inbox;
        WakeHandle*         wh;
    };
    std::mutex mu_;
    std::vector<Sub> subs_;
};

// Meta 수신 스레드: 첫 메타 수신 시 SD 주입
class MetaRxThread {
public:
    MetaRxThread(IEventBus& bus, SpscMailbox<SelfDestructCmd>& sd_mb)
    : bus_(bus), sd_mb_(sd_mb) {
        wh_.cv = &cv_;
        wh_.mu = &mu_;
    }
    void start() {
        running_.store(true);
        bus_.subscribe(Topic::Control, &inbox_, &wh_);
        th_ = std::thread([&]{ run(); });
    }
    void stop() {
        running_.store(false);
        {
            std::lock_guard<std::mutex> lk(mu_);
            cv_.notify_all();
        }
        if (th_.joinable()) th_.join();
        bus_.unsubscribe(&inbox_);
    }
private:
    void run() {
        bool stopped=false;
        while (running_.load()) {
            while (auto ev = inbox_.exchange(nullptr)) {
                if (!stopped && ev->type == EventType::MetaCtrl) {
                    std::cout << "[MetaRx] MetaCtrl observed -> trigger SelfDestruct\n";
                    sd_mb_.push(SelfDestructCmd{++seq_, 9});
                    stopped = true;
                }
            }
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait_for(lk, std::chrono::milliseconds(50));
        }
    }
    IEventBus&                    bus_;
    SpscMailbox<Event>            inbox_;
    SpscMailbox<SelfDestructCmd>& sd_mb_;
    std::thread                   th_;
    std::atomic<bool>             running_{false};
    std::mutex                    mu_;
    std::condition_variable       cv_;
    struct CvWake : public WakeHandle {
        std::condition_variable* cv{};
        std::mutex*              mu{};
        void signal() override { std::lock_guard<std::mutex> lk(*mu); cv->notify_one(); }
    } wh_;
    uint32_t seq_{0};
};

} // namespace flir

int main() {
    using namespace flir;
    using namespace std::chrono;

    // 1) 구성: ControlThread “그대로” 실행
    MockEventBus bus;
    SpscMailbox<SelfDestructCmd> sd_mb;

    TargetFusion   fusion;
    ControllerLR   controller(FRAME_W, DEADZONE_PX); // 로직은 컨트롤러에만
    PrintActuator  act;                               // UART 대체
    CsvLoggerCtrl  logger{"/tmp/ctrl.csv"};

    ControlThread::Config cfg;
    cfg.period_ms     = 50;
    cfg.sd_quiesce_ms = 200;
    cfg.sd_park_ms    = 400;

    ControlThread ctrl(bus, sd_mb, fusion, controller, act, logger, cfg);
    ctrl.start();

    // 2) 메타 수신 스레드: ControlTopic 메타를 보면 SD 발사
    MetaRxThread meta_rx(bus, sd_mb);
    meta_rx.start();

    // 3) 이벤트 생산 (IPC 경로는 Tracking으로 검증)
    //   0.0~2.0s : TRACKING (좌/우 박스 번갈이)
    //   2.0~3.0s : ARUCO 모사 — 컨트롤러에 id 주입(1→RIGHT, 2→LEFT)
    //   3.0~3.5s : ARUCO id=3 → TRACKING 복귀
    std::thread producer([&]{
        uint32_t fseq = 0;

        auto t0 = steady_clock::now();
        while (duration_cast<milliseconds>(steady_clock::now() - t0).count() < 2000) {
            // 왼쪽 박스
            Event eL; eL.type = EventType::Track;
            eL.payload = TrackEvent{ /*box*/{ 40, 40, 20, 20 }, /*score*/0.9f, /*ts*/0, /*frame_seq*/fseq++ };
            bus.push(eL, Topic::Tracking);
            std::this_thread::sleep_for(milliseconds(120));

            // 오른쪽 박스
            Event eR; eR.type = EventType::Track;
            eR.payload = TrackEvent{ /*box*/{ 100, 40, 20, 20 }, /*score*/0.8f, /*ts*/0, /*frame_seq*/fseq++ };
            bus.push(eR, Topic::Tracking);
            std::this_thread::sleep_for(milliseconds(120));
        }

        // 2.0~3.0s: ArUco 동작 모사(IPC는 건드리지 않고 로직만)
        auto t1 = steady_clock::now();
        while (duration_cast<milliseconds>(steady_clock::now() - t1).count() < 1000) {
            controller.test_set_aruco_id(1); // RIGHT
            std::this_thread::sleep_for(milliseconds(200));
            controller.test_set_aruco_id(2); // LEFT
            std::this_thread::sleep_for(milliseconds(200));
        }

        // 3.0~3.5s: id=3 -> TRACKING 복귀
        controller.test_set_aruco_id(3);
        std::this_thread::sleep_for(milliseconds(500));

        // 이후 TRACKING 재개
        auto t2 = steady_clock::now();
        while (duration_cast<milliseconds>(steady_clock::now() - t2).count() < 1000) {
            Event eL; eL.type = EventType::Track;
            eL.payload = TrackEvent{ /*box*/{ 50, 40, 20, 20 }, /*score*/0.9f, /*ts*/0, /*frame_seq*/fseq++ };
            bus.push(eL, Topic::Tracking);
            std::this_thread::sleep_for(milliseconds(120));

            Event eR; eR.type = EventType::Track;
            eR.payload = TrackEvent{ /*box*/{ 110, 40, 20, 20 }, /*score*/0.9f, /*ts*/0, /*frame_seq*/fseq++ };
            bus.push(eR, Topic::Tracking);
            std::this_thread::sleep_for(milliseconds(120));
        }
    });

    // 4) 6초 대기 후 종료 (MetaRx가 첫 MetaCtrl을 보면 SD를 쏘므로 FSM도 함께 검증)
    std::this_thread::sleep_for(std::chrono::seconds(6));
    ctrl.stop();
    ctrl.join();
    meta_rx.stop();
    producer.join();

    std::cout << "[TEST] done\n";
    return 0;
}

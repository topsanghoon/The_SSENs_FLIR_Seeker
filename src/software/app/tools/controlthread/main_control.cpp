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
#include "components/includes/IController.hpp"
#include "components/includes/ActuatorPort.hpp"
#include "components/includes/CsvLoggerCtrl.hpp"
#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/event_bus.hpp"

namespace flir {

static constexpr int   FRAME_W      = 160;
static constexpr float DEADZONE_PX  = 4.0f;

// -----------------------------------------------------
// UART 대체 포트 (터미널 출력)
// -----------------------------------------------------
class PrintActuator final : public IActuatorPort {
public:
    void write_nonblock(const CtrlCmd& cmd) override {
        if (quiesce_.load()) return;
        if (cmd.mode == 99) {
            std::cout << "<< SELF_DESTRUCT SIGNAL >> (mode=99)\n";
            return;
        }
        const char* dir = (cmd.mode < 0 ? "LEFT"
                           : cmd.mode > 0 ? "RIGHT" : "CENTER");
        std::cout << "[UART] DIR=" << dir << " (err=" << cmd.p1 << ")\n";
    }
    void set_quiesce(bool q) override {
        quiesce_.store(q);
        std::cout << "[UART] set_quiesce=" << (q?"true":"false") << "\n";
    }
    void stop_io() override {
        std::cout << "[UART] stop_io\n";
    }
private:
    std::atomic<bool> quiesce_{false};
};

// -----------------------------------------------------
// EventBus(Mock)
// -----------------------------------------------------
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

// -----------------------------------------------------
// MetaRxThread: 20초 경과 시 자폭 명령 발행
// -----------------------------------------------------
class MetaRxThread {
public:
    MetaRxThread(SpscMailbox<SelfDestructCmd>& sd_mb)
    : sd_mb_(sd_mb) {}

    void start() {
        running_.store(true);
        th_ = std::thread([&]{ run(); });
    }
    void stop() {
        running_.store(false);
        if (th_.joinable()) th_.join();
    }

private:
    void run() {
        using namespace std::chrono;
        auto t0 = steady_clock::now();
        while (running_.load()) {
            auto elapsed = duration_cast<seconds>(steady_clock::now() - t0).count();
            if (elapsed >= 20 && !triggered_) {
                std::cout << "\n[MetaRx] === SelfDestruct triggered ===\n";
                sd_mb_.push(SelfDestructCmd{++seq_, 9});
                triggered_ = true;
            }
            std::this_thread::sleep_for(milliseconds(200));
        }
    }

    SpscMailbox<SelfDestructCmd>& sd_mb_;
    std::thread th_;
    std::atomic<bool> running_{false};
    uint32_t seq_{0};
    bool triggered_ = false;
};

} // namespace flir

// -----------------------------------------------------
// MAIN TEST
// -----------------------------------------------------
int main() {
    using namespace flir;
    using namespace std::chrono;

    MockEventBus bus;
    SpscMailbox<SelfDestructCmd> sd_mb;

    TargetFusion   fusion;
    ControllerLR   controller(FRAME_W, DEADZONE_PX);
    PrintActuator  act;
    CsvLoggerCtrl  logger{"/tmp/ctrl.csv"};

    ControlThread::Config cfg;
    cfg.period_ms     = 50;
    cfg.sd_quiesce_ms = 200;
    cfg.sd_park_ms    = 400;

    ControlThread ctrl(bus, sd_mb, fusion, controller, act, logger, cfg);
    ctrl.start();

    MetaRxThread meta_rx(sd_mb);
    meta_rx.start();

    std::thread producer([&]{
        uint32_t fseq = 0;
        auto t0 = steady_clock::now();

        while (true) {
            auto elapsed = duration_cast<seconds>(steady_clock::now() - t0).count();

            if (elapsed < 10) {
                // ---- ① 탐지 단계 (좌우 번갈이) ----
                std::cout << "[Phase1] DETECTION active\n";
                Event eL; eL.type = EventType::Track;
                eL.payload = TrackEvent{{ 30,40,20,20 }, 0.9f, 0, fseq++};
                bus.push(eL, Topic::Tracking);
                std::this_thread::sleep_for(milliseconds(150));

                Event eR; eR.type = EventType::Track;
                eR.payload = TrackEvent{{ 110,40,20,20 }, 0.8f, 0, fseq++};
                bus.push(eR, Topic::Tracking);
                std::this_thread::sleep_for(milliseconds(150));
            }
            else if (elapsed < 20) {
                // ---- ② 추적 단계 (중심 근처 박스 유지) ----
                std::cout << "[Phase2] TRACKING stable\n";
                Event eC; eC.type = EventType::Track;
                eC.payload = TrackEvent{{ 70,40,20,20 }, 0.95f, 0, fseq++};
                bus.push(eC, Topic::Tracking);
                std::this_thread::sleep_for(milliseconds(200));
            }
            else break;
        }
    });

    // 전체 시나리오 25초간 유지
    std::this_thread::sleep_for(seconds(25));
    ctrl.stop();
    ctrl.join();
    meta_rx.stop();
    producer.join();

    std::cout << "\n[TEST] Simulation completed\n";
    return 0;
}

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

    try { // <<<<<<<<<<<< UART 열기 실패에 대비해 try 블록 추가
        MockEventBus bus;
        SpscMailbox<SelfDestructCmd> sd_mb;

        TargetFusion  fusion;
        ControllerLR  controller(FRAME_W, DEADZONE_PX);
        
        // --- 여기를 수정 ---
        // 기존 코드: PrintActuator  act;
        // 새 코드:
        UART_ActuatorPort act("/dev/ttyUL0", B9600);
        // --------------------

        CsvLoggerCtrl logger{"/tmp/ctrl.csv"};

        ControlThread::Config cfg;
        // ... (cfg 설정) ...

        ControlThread ctrl(bus, sd_mb, fusion, controller, act, logger, cfg);
        ctrl.start();

        MetaRxThread meta_rx(sd_mb);
        meta_rx.start();

        std::thread producer([&]{
        uint32_t fseq = 0;
        auto t0 = steady_clock::now();

        while (true) {
            auto elapsed = duration_cast<seconds>(steady_clock::now() - t0).count();
            

            if (elapsed < 5) {
                // ---- 중기 단계 (좌우 번갈이) ----
                std::cout << "[Phase1] Aruco active\n";
                Event a1; a1.type = EventType::Aruco;
                a1.payload = ArucoEvent{1, { {{100, 200}, {300, 400}}}, { 76, 40,20,20 }, 0};
                bus.push(a1, Topic::Aruco);
                std::this_thread::sleep_for(milliseconds(1000));

                Event a2; a2.type = EventType::Aruco;
                a2.payload = ArucoEvent{1, { {{1, 1}, {1, 1}}}, { 64, 40,20,20 }, 0};
                bus.push(a2, Topic::Aruco);
                std::this_thread::sleep_for(milliseconds(1000));
            }
            else if (elapsed < 10) {
                Event e1; e1.type = EventType::Track;
                e1.payload = TrackEvent{{ 74,40,20,20 }, 0.95f, 0, fseq++};
                bus.push(e1, Topic::Tracking);
                std::this_thread::sleep_for(milliseconds(500));
                Event e11; e11.type = EventType::Track;
                e11.payload = TrackEvent{{ 65,40,20,20 }, 0.95f, 0, fseq++};
                bus.push(e11, Topic::Tracking);
                std::this_thread::sleep_for(milliseconds(500));
            }
            else if (elapsed < 15) {
                Event e2; e2.type = EventType::Track;
                e2.payload = TrackEvent{{ 100,40,20,20 }, 0.95f, 0, fseq++};
                bus.push(e2, Topic::Tracking);
                std::this_thread::sleep_for(milliseconds(500));
                Event e22; e22.type = EventType::Track;
                e22.payload = TrackEvent{{ 40,40,20,20 }, 0.95f, 0, fseq++};
                bus.push(e22, Topic::Tracking);
                std::this_thread::sleep_for(milliseconds(500));
            }
            else if (elapsed < 20) {
                Event e3; e3.type = EventType::Track;
                e3.payload = TrackEvent{{ 140,40,20,20 }, 0.95f, 0, fseq++};
                bus.push(e3, Topic::Tracking);
                std::this_thread::sleep_for(milliseconds(500));
                Event e33; e33.type = EventType::Track;
                e33.payload = TrackEvent{{ 0,40,20,20 }, 0.95f, 0, fseq++};
                bus.push(e33, Topic::Tracking);
                std::this_thread::sleep_for(milliseconds(500));
            }
            else break;

            // if (elapsed < 5) {
            //     // ---- 중기 단계 (좌우 번갈이) ----
            //     std::cout << "[Phase1] Aruco active\n";
            //     controller.test_set_aruco_id(1);
            //     Event a1; a1.type = EventType::Aruco;
            //     a1.payload = ArucoEvent{1, { {{1, 1}, {1, 1}}}, { 30,40,20,20 }, 0};
            //     bus.push(a1, Topic::Aruco);
            //     std::this_thread::sleep_for(milliseconds(150));

            //     Event a2; a2.type = EventType::Aruco;
            //     controller.test_set_aruco_id(3);
            //     a2.payload = ArucoEvent{1, { {{1, 1}, {1, 1}}}, { 30,40,20,20 }, 0};
            //     bus.push(a2, Topic::Aruco);
            //     std::this_thread::sleep_for(milliseconds(150));
            // }
            // else if (elapsed < 15) {
            //     // ---- 탐지 단계 (좌우 번갈이) ----
            //     std::cout << "[Phase2] DETECTION active\n";
            //     controller.test_force_tracking();
            //     Event eL; eL.type = EventType::Track;
            //     eL.payload = TrackEvent{{ 30,40,20,20 }, 0.9f, 0, fseq++};
            //     bus.push(eL, Topic::Tracking);
            //     std::this_thread::sleep_for(milliseconds(150));

            //     Event eR; eR.type = EventType::Track;
            //     eR.payload = TrackEvent{{ 110,40,20,20 }, 0.8f, 0, fseq++};
            //     bus.push(eR, Topic::Tracking);
            //     std::this_thread::sleep_for(milliseconds(150));
            // }
            // else if (elapsed < 20) {
            //     // ---- 추적 단계 (중심 근처 박스 유지) ----
            //     std::cout << "[Phase3] TRACKING stable\n";
            //     Event eC1; eC1.type = EventType::Track;
            //     eC1.payload = TrackEvent{{ 70,40,20,20 }, 0.95f, 0, fseq++};
            //     bus.push(eC1, Topic::Tracking);
            //     std::this_thread::sleep_for(milliseconds(100));
            //     std::cout << "[Phase3] TRACKING stable\n";
            //     Event eC2; eC2.type = EventType::Track;
            //     eC2.payload = TrackEvent{{ 73,40,20,20 }, 0.95f, 0, fseq++};
            //     bus.push(eC2, Topic::Tracking);
            //     std::this_thread::sleep_for(milliseconds(100));
            //     std::cout << "[Phase3] TRACKING stable\n";
            //     Event eC3; eC3.type = EventType::Track;
            //     eC3.payload = TrackEvent{{ 66,40,20,20 }, 0.95f, 0, fseq++};
            //     bus.push(eC3, Topic::Tracking);
            //     std::this_thread::sleep_for(milliseconds(100));
            // }
            // else break;
        }
    });

    // 전체 시나리오 25초간 유지
    std::this_thread::sleep_for(seconds(25));
    ctrl.stop();
    ctrl.join();
    meta_rx.stop();
    producer.join();

    } catch (const std::exception& e) { // <<<<<<<<<<<< 예외 처리 catch 블록
        std::cerr << "FATAL ERROR: " << e.what() << std::endl;
        return 1;
    }

    std::cout << "\n[TEST] Simulation completed\n";
    return 0;
}

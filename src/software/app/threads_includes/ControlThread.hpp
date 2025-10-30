#pragma once
#include <atomic>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <chrono>

#include "ipc/ipc_types.hpp"     // Event, EventType, MetaCtrlEvent, Topics, SelfDestructCmd
#include "ipc/mailbox.hpp"       // SpscMailbox
#include "ipc/event_bus.hpp"     // IEventBus, WakeHandle

#include "components/includes/ControlDTO.hpp"   // CtrlCmd
#include "components/includes/TargetFusion.hpp"
#include "components/includes/IController.hpp"
#include "components/includes/ActuatorPort.hpp"

// 로그/타임 (단일 CSV 싱크 매크로)
#include "util/telemetry.hpp"    // CSV_LOG_SIMPLE
#include "util/time_util.hpp"    // now_ns()

namespace flir {

class ControlThread {
public:
    struct Config {
        int period_ms     = 20;   // 50Hz (기존 주석은 200이었지만 50Hz면 20ms 주기)
        int sd_quiesce_ms = 200;
        int sd_park_ms    = 400;
    };

    // 기본 생성자 제거 → 반드시 Config 전달
    ControlThread(IEventBus&                    bus,
                  SpscMailbox<SelfDestructCmd>& sd_mb,
                  TargetFusion&                 fusion,
                  IController&                  controller,
                  IActuatorPort&                act,
                  Config                        cfg);

    void start();
    void stop();
    void join();

private:
    using clock_t = std::chrono::steady_clock;

    // 협력자
    IEventBus&                    bus_;
    SpscMailbox<Event>            inbox_;     // Tracking/Aruco 입력
    SpscMailbox<SelfDestructCmd>& sd_mb_;     // 자폭 명령 입력
    TargetFusion&                 fusion_;
    IController&                  controller_;
    IActuatorPort&                act_;
    Config                        cfg_;

    // 스레드 관리
    std::thread             th_;
    std::atomic<bool>       running_{false};
    std::mutex              m_;
    std::condition_variable cv_;

    // 상태
    enum class Mode   { RUN, SHUTDOWN };
    enum class SdPhase{ SD_IDLE, SD_QUIESCE, SD_DONE };

    Mode       mode_  = Mode::RUN;
    SdPhase    phase_ = SdPhase::SD_IDLE;
    uint32_t   sd_seq_seen_ = 0;
    clock_t::time_point t_phase_start_{};

    clock_t::time_point next_tick_tp_{clock_t::now()};

    void run();
    bool ready_to_wake();

    void drain_events();
    void handle_self_destruct();

    void tick_run_mode();
    void step_shutdown_fsm();

    // Wake handle (EventBus 신호용)
    struct CvWakeHandle : public WakeHandle {
        std::condition_variable* cv{};
        std::mutex*              mu{};
        void signal() override {
            std::lock_guard<std::mutex> lk(*mu);
            cv->notify_one();
        }
    } wake_;
};

} // namespace flir

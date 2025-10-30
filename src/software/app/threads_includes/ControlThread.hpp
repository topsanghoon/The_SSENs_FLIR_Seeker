#pragma once
#include <atomic>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/event_bus.hpp"

#include "components/includes/ControlDTO.hpp"
#include "components/includes/TargetFusion.hpp"
#include "components/includes/IController.hpp"
#include "components/includes/ActuatorPort.hpp"
#include "components/includes/CsvLoggerCtrl.hpp"

namespace flir {

class ControlThread {
public:
    struct Config {
        int period_ms     = 200;   // 제어 주기 (50Hz)
        int sd_quiesce_ms = 200;
        int sd_park_ms    = 400;
    };

    // 기본 생성자 제거 → 반드시 Config 전달
    ControlThread(IEventBus&                    bus,
                  SpscMailbox<SelfDestructCmd>& sd_mb,
                  TargetFusion&                 fusion,
                  IController&                  controller,
                  IActuatorPort&                act,
                  CsvLoggerCtrl&                logger,
                  Config                        cfg);

    void start();
    void stop();
    void join();

private:
    using clock_t = std::chrono::steady_clock;

    // 협력자
    IEventBus&                    bus_;
    SpscMailbox<Event>            inbox_;
    SpscMailbox<SelfDestructCmd>& sd_mb_;
    TargetFusion&                 fusion_;
    IController&                  controller_;
    IActuatorPort&                act_;
    CsvLoggerCtrl&                log_;
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
    std::chrono::steady_clock::time_point t_phase_start_{};

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

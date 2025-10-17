#pragma once
#include <atomic>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/event_bus.hpp"

// 컴포넌트 의존
#include "components/includes/ControlDTO.hpp"     // CtrlCmd, SelfDestructCmd
#include "components/includes/TargetFusion.hpp"   // TargetFusion
#include "components/includes/IController.hpp"    // IController
#include "components/includes/ActuatorPort.hpp"   // IActuatorPort
#include "components/includes/CsvLoggerCtrl.hpp"  // CsvLoggerCtrl

namespace flir {

class ControlThread {
public:
    struct Config {
        int   period_ms     = 20;   // 주기 제어 틱 (50Hz)
        int   sd_quiesce_ms = 100;
        int   sd_park_ms    = 500;
    };

    ControlThread(IEventBus&                  bus,
                  SpscMailbox<SelfDestructCmd>& sd_mb,   // 자폭/안전정지 명령
                  TargetFusion&               fusion,
                  IController&                controller,
                  IActuatorPort&              act,
                  CsvLoggerCtrl               logger,
                  Config                      cfg = {});

    void start();
    void stop();
    void join();

private:
    // 협력자
    IEventBus&                  bus_;
    SpscMailbox<Event>          inbox_;   // EVT_BUS 구독용 inbox (Track/Aruco)
    SpscMailbox<SelfDestructCmd>& sd_mb_;
    TargetFusion&               fusion_;
    IController&                controller_;
    IActuatorPort&              act_;
    CsvLoggerCtrl               log_;
    Config                      cfg_;

    // 스레드/동기화
    std::thread                 th_;
    std::atomic<bool>           running_{false};
    std::mutex                  m_;
    std::condition_variable     cv_;

    // 상태
    enum class Mode   { RUN, SHUTDOWN };
    enum class SdPhase{ SD_IDLE, SD_QUIESCE, SD_PARK, SD_STOP_IO, SD_DONE };

    Mode       mode_  = Mode::RUN;
    SdPhase    phase_ = SdPhase::SD_IDLE;
    uint32_t   sd_seq_seen_ = 0;
    std::chrono::steady_clock::time_point t_phase_start_{};

    // 내부 헬퍼
    void run();

    // 대기조건: 이벤트/자폭/주기
    bool ready_to_wake();

    // 이벤트 처리
    void drain_events(); // inbox_에서 Track/Aruco 흡수 → fusion_
    void handle_self_destruct();

    // 모드별 동작
    void tick_run_mode();        // 평시 제어계산 + 출력
    void step_shutdown_fsm();    // 단계적 정지 FSM

    // wakehandle (CV 기반) — EventBus가 push할 때 깨우게 전달
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

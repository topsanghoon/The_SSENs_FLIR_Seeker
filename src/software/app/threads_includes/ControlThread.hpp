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

#include "util/telemetry.hpp"
#include "util/time_util.hpp"

// ★ 추가
#include "guidance_mode.hpp"
#include "main_config.hpp"  // GuidanceConfig를 사용하기 위함

namespace flir {

class ControlThread {
public:
    struct Config {
        int period_ms     = 20;   // 50Hz
        int sd_quiesce_ms = 200;
        int sd_park_ms    = 400;

        // ★ 전환 기준(상위 AppConfig.guidance 주입값을 그대로 복사 보관)
        GuidanceConfig guidance{};
        // EO 프레임 크기(비율 판단용). AppConfig.eo_tx.frame를 넘겨 유지
        int eo_w{640};
        int eo_h{480};
    };

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
    SpscMailbox<Event>            inbox_;
    SpscMailbox<SelfDestructCmd>& sd_mb_;
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

    // ★ 중기→종말 전환 판별 상태
    int                   big_cnt_ = 0;
    bool                  big_reached_ = false;
    clock_t::time_point   last_seen_tp_{};

    void run();
    bool ready_to_wake();

    void drain_events();
    void handle_self_destruct();

    void tick_run_mode();
    void step_shutdown_fsm();

    // ★ 전환 관련 헬퍼
    void on_aruco_for_transition(int id, const cv::Rect& box, uint64_t ts_ns);
    bool is_big_enough(int bw, int bh) const;

    // Wake handle
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

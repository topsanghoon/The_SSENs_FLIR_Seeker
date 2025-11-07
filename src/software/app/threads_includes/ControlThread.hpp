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

// ★ 전환/가이던스
#include "guidance_mode.hpp"
#include "main_config.hpp"  // GuidanceConfig

namespace flir {

class ControlThread {
public:
    struct Config {
        // period_ms는 이제 "주기 송신"이 아니라, 내부 FSM 등에 쓰일 수 있는 보조 시간 단위로만 남겨둠
        int period_ms     = 20;   // 50Hz 상수, 필요시 내부 계산에만 사용
        int sd_quiesce_ms = 200;
        int sd_park_ms    = 400;

        // ★ 전환 기준(상위 AppConfig.guidance 주입값을 복사 보관)
        GuidanceConfig guidance{};
        // EO 프레임 크기(비율 판단용)
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

    using ShutdownCb = std::function<void()>;
    void set_on_shutdown(ShutdownCb cb) { on_shutdown_ = std::move(cb); }

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

    // 이벤트 드리븐: 최근 드레인에서 “새 센싱 이벤트가 있었는지”
    bool had_new_sensing_evt_ = false;

    // ★ 중기→종말 전환 판별 상태
    int                 big_cnt_ = 0;
    bool                big_reached_ = false;
    clock_t::time_point last_seen_tp_{};
    clock_t::time_point big_reached_tp_{};

    void run();

    // ready 조건: 이벤트 도착(Tracking/Aruco) 또는 SD 수신
    bool ready_to_wake();

    // 센싱 이벤트를 모두 비우고, 새 이벤트가 있었으면 true를 리턴
    bool drain_events();

    // SD 처리
    void handle_self_destruct();

    // RUN 모드: 센싱 이벤트가 있었고 타깃이 유효할 때만 송신
    void maybe_emit_control_if_target();

    // SD FSM
    void step_shutdown_fsm();

    // ★ 전환 관련 헬퍼
    void on_aruco_for_transition(int id, const cv::Rect& box, uint64_t ts_ns);
    bool is_big_enough(int bw, int bh) const;

    // 타깃 유무(간단 게이트)
    static inline bool has_target_box(const cv::Rect& b) {
        return b.width > 0 && b.height > 0;
    }

    // Wake handle (EventBus가 signal 호출)
    struct CvWakeHandle : public WakeHandle {
        std::condition_variable* cv{};
        std::mutex*              mu{};
        void signal() override {
            std::lock_guard<std::mutex> lk(*mu);
            cv->notify_one();
        }
    } wake_;

    ShutdownCb on_shutdown_;
};


} // namespace flir

#include "threads_includes/ControlThread.hpp"
#include "util/time_util.hpp"
#include <cassert>

namespace flir {

using clock_t = std::chrono::steady_clock;
static constexpr const char* TAG = "Control";

// =====================================================
// ctor
// =====================================================
ControlThread::ControlThread(IEventBus& bus,
                             SpscMailbox<SelfDestructCmd>& sd_mb,
                             TargetFusion& fusion,
                             IController& controller,
                             IActuatorPort& act,
                             Config cfg)
: bus_(bus)
, sd_mb_(sd_mb)
, fusion_(fusion)
, controller_(controller)
, act_(act)
, cfg_(cfg)
{
    wake_.cv = &cv_;
    wake_.mu = &m_;
    next_tick_tp_ = clock_t::now() + std::chrono::milliseconds(cfg_.period_ms);
}

// =====================================================
void ControlThread::start() {
    if (running_.exchange(true)) return;

    bus_.subscribe(Topic::Tracking, &inbox_, &wake_);
    bus_.subscribe(Topic::Aruco,    &inbox_, &wake_);

    th_ = std::thread(&ControlThread::run, this);

    CSV_LOG_SIMPLE("Ctrl", "THREAD_START", 0, 0,0,0,0, "");
}

void ControlThread::stop() {
    if (!running_.load()) return;
    running_.store(false);
    {
        std::lock_guard<std::mutex> lk(m_);
        cv_.notify_all();
    }
}

void ControlThread::join() {
    if (th_.joinable()) th_.join();
    bus_.unsubscribe(&inbox_);
    CSV_LOG_SIMPLE("Ctrl", "THREAD_STOP", 0, 0,0,0,0, "");
}

// =====================================================
// ready_to_wake()
//  - 새 이벤트 / 새 자폭 / 주기 틱 중 하나만 있어도 깸
// =====================================================
bool ControlThread::ready_to_wake() {
    const bool has_evt = (inbox_.latest_seq() > fusion_.last_event_seq());
    const bool has_sd  = (sd_mb_.latest_seq() > sd_seq_seen_);
    const bool tick    = (clock_t::now() >= next_tick_tp_);
    return !running_.load() ? true : (has_evt || has_sd || tick);
}

// =====================================================
// run(): 주기 루프
// =====================================================
void ControlThread::run() {
    while (running_.load()) {
        {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait_until(lk, next_tick_tp_, [&]{ return ready_to_wake(); });
        }
        if (!running_.load()) break;

        // 입력 처리
        handle_self_destruct();
        drain_events();

        // 주기 제어 or SHUTDOWN FSM
        if (mode_ == Mode::RUN) {
            tick_run_mode();
        } else {
            step_shutdown_fsm();
        }

        next_tick_tp_ = clock_t::now() + std::chrono::milliseconds(cfg_.period_ms);
    }
}

// =====================================================
// drain_events(): Tracking / ArUco → Fusion 갱신
// =====================================================
void ControlThread::drain_events() {
    while (auto ev = inbox_.exchange(nullptr)) {
        switch (ev->type) {
            case EventType::Track: {
                const auto& x = std::get<TrackEvent>(ev->payload);
                fusion_.update_with_track(x.box, x.score, x.ts, x.frame_seq);
                CSV_LOG_SIMPLE("Ctrl", "IN_TRACK", x.frame_seq,
                               x.box.x, x.box.y, x.box.width, x.box.height, "");
            } break;
            case EventType::Aruco: {
                const auto& x = std::get<ArucoEvent>(ev->payload);
                fusion_.update_with_marker(x.id, x.box, x.ts);
                CSV_LOG_SIMPLE("Ctrl", "IN_ARUCO", (uint64_t)x.id,
                               x.box.x, x.box.y, x.box.width, x.box.height, "");
            } break;
            default: break;
        }
        fusion_.bump_last_event_seq();
    }
}

// =====================================================
// handle_self_destruct(): 자폭 명령 수신 시 FSM 진입
// =====================================================
void ControlThread::handle_self_destruct() {
    if (sd_mb_.has_new(sd_seq_seen_)) {
        if (auto sd = sd_mb_.exchange(nullptr)) {
            sd_seq_seen_ = sd->seq;
            mode_  = Mode::SHUTDOWN;
            phase_ = SdPhase::SD_QUIESCE;
            t_phase_start_ = clock_t::now();

            CSV_LOG_SIMPLE("Ctrl", "SD_REQ", sd->seq, (double)sd->level, 0,0,0, "");
        }
    }
}

// =====================================================
// tick_run_mode(): 정상 제어 주기
//  - Controller로부터 명령 계산
//  - Meta_TxThread가 보낼 수 있게 EventBus에 MetaCtrlEvent 게시
//  - ActuatorPort로 비차단 송신
// =====================================================
void ControlThread::tick_run_mode() {
    CtrlCmd cmd = controller_.solve(fusion_);
    const uint64_t ts = flir::now_ns_steady();  // ★ 교체

    Event ev{ EventType::MetaCtrl, MetaCtrlEvent{ cmd.to_int(), ts } };
    bus_.push(ev, Topic::Control);

    act_.write_nonblock(cmd);
    CSV_LOG_SIMPLE("Ctrl", "CTRL_OUT", 0, (double)cmd.to_int(), 0,0,0, "");
}


// =====================================================
// step_shutdown_fsm(): 단순화된 자폭 시퀀스
//  - 아두이노가 실제 정지 수행
//  - Zynq는 자폭 신호만 송신 (특수 명령)
// =====================================================
void ControlThread::step_shutdown_fsm() {
    if (phase_ == SdPhase::SD_QUIESCE) {
        fusion_.set_quiesce(true);
        act_.set_quiesce(true);

        CtrlCmd sd = CtrlCmd::make_self_destruct();
        const uint64_t ts = flir::now_ns_steady();  // ★ 교체

        Event ev{ EventType::MetaCtrl, MetaCtrlEvent{ sd.to_int(), ts } };
        bus_.push(ev, Topic::Control);

        act_.write_nonblock(sd);
        CSV_LOG_SIMPLE("Ctrl", "SELF_DESTRUCT_SENT", 0, (double)sd.to_int(), 0,0,0, "");

        phase_ = SdPhase::SD_DONE;
        mode_  = Mode::SHUTDOWN;
    }
}

} // namespace flir

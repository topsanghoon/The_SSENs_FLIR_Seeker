#include "threads_includes/ControlThread.hpp"
#include <cassert>

namespace flir {

using clock_t = std::chrono::steady_clock;

ControlThread::ControlThread(IEventBus& bus,
                             SpscMailbox<SelfDestructCmd>& sd_mb,
                             TargetFusion& fusion,
                             IController& controller,
                             IActuatorPort& act,
                             CsvLoggerCtrl& logger)
: ControlThread(bus, sd_mb, fusion, controller, act, logger, Config{}) {} // 위임

ControlThread::ControlThread(IEventBus& bus,
                             SpscMailbox<SelfDestructCmd>& sd_mb,
                             TargetFusion& fusion,
                             IController& controller,
                             IActuatorPort& act,
                             CsvLoggerCtrl& logger,
                             Config cfg)
: bus_(bus)
, sd_mb_(sd_mb)
, fusion_(fusion)
, controller_(controller)
, act_(act)
, log_(logger)
, cfg_(cfg)
{
    wake_.cv = &cv_;
    wake_.mu = &m_;
    next_tick_tp_ = clock_t::now() + std::chrono::milliseconds(cfg_.period_ms);
}

void ControlThread::start() {
    running_.store(true);
    bus_.subscribe(Topic::Tracking, &inbox_, &wake_);
    bus_.subscribe(Topic::Aruco,    &inbox_, &wake_);
    th_ = std::thread(&ControlThread::run, this);
}

void ControlThread::stop() {
    running_.store(false);
    {
        std::lock_guard<std::mutex> lk(m_);
        cv_.notify_all();
    }
}

void ControlThread::join() {
    if (th_.joinable()) th_.join();
    bus_.unsubscribe(&inbox_);
}

bool ControlThread::ready_to_wake() {
    const bool has_evt = (inbox_.latest_seq() > fusion_.last_event_seq());
    const bool has_sd  = (sd_mb_.latest_seq() > sd_seq_seen_);
    const bool tick    = (clock_t::now() >= next_tick_tp_);
    return !running_.load() ? true : (has_evt || has_sd || tick);
}

void ControlThread::run() {
    while (running_.load()) {
        {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait_until(lk, next_tick_tp_, [&]{ return ready_to_wake(); });
        }
        if (!running_.load()) break;

        handle_self_destruct();
        drain_events();

        if (mode_ == Mode::RUN) {
            tick_run_mode();
        } else {
            step_shutdown_fsm();
        }

        next_tick_tp_ = clock_t::now() + std::chrono::milliseconds(cfg_.period_ms);
    }
}

void ControlThread::drain_events() {
    while (auto ev = inbox_.exchange(nullptr)) {
        switch (ev->type) {
            case EventType::Track: {
                const auto& x = std::get<TrackEvent>(ev->payload);
                fusion_.update_with_track(x.box, x.score, x.ts, x.frame_seq);
            } break;
            case EventType::Aruco: {
                const auto& x = std::get<ArucoEvent>(ev->payload);
                fusion_.update_with_marker(x.id, x.box, x.ts);
            } break;
            default: break;
        }
        fusion_.bump_last_event_seq();
    }
}

void ControlThread::handle_self_destruct() {
    if (sd_mb_.has_new(sd_seq_seen_)) {
        if (auto sd = sd_mb_.exchange(nullptr)) {
            sd_seq_seen_ = sd->seq;
            mode_  = Mode::SHUTDOWN;
            phase_ = SdPhase::SD_QUIESCE;
            t_phase_start_ = clock_t::now();   // 오타 수정됨
            log_.sd_req(sd->seq, sd->level);
        }
    }
}

void ControlThread::tick_run_mode() {
    CtrlCmd cmd = controller_.solve(fusion_);
    Event ev{ EventType::MetaCtrl, MetaCtrlEvent{ cmd.to_int(), controller_.now_ns() } };
    bus_.push(ev, Topic::Control);
    act_.write_nonblock(cmd);
    log_.ctrl_out(cmd);
}

void ControlThread::step_shutdown_fsm() {
    using std::chrono::milliseconds;
    auto elapsed = std::chrono::duration_cast<milliseconds>(clock_t::now() - t_phase_start_).count();

    switch (phase_) {
        case SdPhase::SD_QUIESCE: {
            fusion_.set_quiesce(true);
            act_.set_quiesce(true);
            log_.phase("QUIESCE");
            if (elapsed > cfg_.sd_quiesce_ms) {
                phase_ = SdPhase::SD_PARK; t_phase_start_ = clock_t::now();
            }
        } break;
        case SdPhase::SD_PARK: {
            CtrlCmd safe = CtrlCmd::safe_pose();
            act_.write_nonblock(safe);
            log_.phase("PARK");
            if (elapsed > cfg_.sd_park_ms) {
                phase_ = SdPhase::SD_STOP_IO; t_phase_start_ = clock_t::now();
            }
        } break;
        case SdPhase::SD_STOP_IO: {
            act_.stop_io();
            fusion_.drain_for_ms(100);
            log_.phase("STOP_IO");
            phase_ = SdPhase::SD_DONE;
        } break;
        case SdPhase::SD_DONE: {
            log_.sd_done();
        } break;
        default: break;
    }
}

} // namespace flir

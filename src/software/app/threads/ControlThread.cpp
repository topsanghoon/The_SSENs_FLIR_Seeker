#include "threads_includes/ControlThread.hpp"
#include <cassert>

namespace flir {

using clock_t = std::chrono::steady_clock;

ControlThread::ControlThread(IEventBus& bus,
                             SpscMailbox<SelfDestructCmd>& sd_mb,
                             TargetFusion& fusion,
                             IController& controller,
                             IActuatorPort& act,
                             CsvLoggerCtrl logger,
                             Config cfg)
: bus_(bus)
, sd_mb_(sd_mb)
, fusion_(fusion)
, controller_(controller)
, act_(act)
, log_(std::move(logger))
, cfg_(cfg)
{
    wake_.cv = &cv_;
    wake_.mu = &m_;
}

void ControlThread::start() {
    running_.store(true);
    // EVT_BUS 구독: Tracking/Aruco → 하나의 inbox_로
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
    // (주의) 여기서는 lock 잡힌 상태에서만 호출
    // 조건: 새 EVT, 새 SD, 주기 틱 도래
    const bool has_evt = (inbox_.latest_seq() > fusion_.last_event_seq());
    const bool has_sd  = (sd_mb_.latest_seq() > sd_seq_seen_);

    static auto next_tick = clock_t::now();
    auto now = clock_t::now();
    bool tick = false;
    if (now >= next_tick) {
        tick = true;
        next_tick = now + std::chrono::milliseconds(cfg_.period_ms);
    }
    return !running_.load() ? true : (has_evt || has_sd || tick);
}

void ControlThread::run() {
    // 주기 틱 기준 시각
    auto next_wakeup = clock_t::now() + std::chrono::milliseconds(cfg_.period_ms);

    while (running_.load()) {
        // 1) 대기
        {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait_until(lk, next_wakeup, [&]{ return ready_to_wake(); });
        }
        if (!running_.load()) break;

        // 2) 우선 순위: SD 요청 흡수(모드 전환)
        handle_self_destruct();

        // 3) 이벤트 소화 (Track/Aruco)
        drain_events();

        // 4) 모드별 동작
        if (mode_ == Mode::RUN) {
            tick_run_mode();
        } else {
            step_shutdown_fsm();
        }

        // 다음 주기 예약
        next_wakeup = clock_t::now() + std::chrono::milliseconds(cfg_.period_ms);
    }
}

void ControlThread::drain_events() {
    // inbox_에서 이벤트 모두 소비
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
        fusion_.bump_last_event_seq(); // 간단: 소비마다 내부 seq 상승 (구현에 맞게)
    }
}

void ControlThread::handle_self_destruct() {
    if (sd_mb_.has_new(sd_seq_seen_)) {
        if (auto sd = sd_mb_.exchange(nullptr)) {
            sd_seq_seen_ = sd->seq;
            // 모드 전환
            mode_  = Mode::SHUTDOWN;
            phase_ = SdPhase::SD_QUIESCE;
            t_phase_start_ = c                                                                                                                                                                                                                                                                                                                                                                                               lock_t::now();
            log_.sd_req(sd->seq, sd->level);
        }
    }
}

void ControlThread::tick_run_mode() {
    // 1) 목표(추적/마커) 기반 제어해석
    CtrlCmd cmd = controller_.solve(fusion_);

    // 2) 출력
    //   2-1) 이벤트 버스(지상국/메타 전송용)
    Event ev{ EventType::MetaCtrl, MetaCtrlEvent{ cmd.to_int(), /*ts*/ controller_.now_ns() } };
    bus_.push(ev, Topic::Control);

    //   2-2) 구동부(UART 등) 비동기 송신
    act_.write_nonblock(cmd);

    // 3) 로깅
    log_.ctrl_out(cmd);
}

void ControlThread::step_shutdown_fsm() {
    using std::chrono::milliseconds;
    auto elapsed = std::chrono::duration_cast<milliseconds>(clock_t::now() - t_phase_start_).count();

    switch (phase_) {
        case SdPhase::SD_QUIESCE: {
            // 캡처/Tx에 조용히 하라고 신호 (여기선 컴포넌트 메서드로 추상화했다고 가정)
            fusion_.set_quiesce(true);
            act_.set_quiesce(true);
            log_.phase("QUIESCE");
            if (elapsed > cfg_.sd_quiesce_ms) {
                phase_ = SdPhase::SD_PARK; t_phase_start_ = clock_t::now();
            }
        } break;

        case SdPhase::SD_PARK: {
            // 안전자세 명령 반복
            CtrlCmd safe = CtrlCmd::safe_pose();
            act_.write_nonblock(safe);
            log_.phase("PARK");
            if (elapsed > cfg_.sd_park_ms) {
                phase_ = SdPhase::SD_STOP_IO; t_phase_start_ = clock_t::now();
            }
        } break;

        case SdPhase::SD_STOP_IO: {
            // I/O를 논리적으로 정지(이벤트는 상위에서 조인/정리)
            act_.stop_io();
            fusion_.drain_for_ms(100);
            log_.phase("STOP_IO");
            phase_ = SdPhase::SD_DONE;
        } break;

        case SdPhase::SD_DONE: {
            log_.sd_done();
            // 여기서 유지 또는 상위 종료 루틴 대기
        } break;

        default: break;
    }
}

} // namespace flir

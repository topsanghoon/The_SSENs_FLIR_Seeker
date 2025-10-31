#include "threads_includes/ControlThread.hpp"
#include "util/time_util.hpp"
#include <cassert>

namespace flir {

using clock_t = std::chrono::steady_clock;
static constexpr const char* TAG = "Control";

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

    // ★ 초기 유도 단계 설정
    GuidanceState::phase().store(cfg_.guidance.default_phase);
    big_cnt_ = 0; big_reached_ = false;
    last_seen_tp_ = clock_t::now();
}

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
                CSV_LOG_SIMPLE("Ctrl", "IN_TRACK", x.frame_seq,
                               x.box.x, x.box.y, x.box.width, x.box.height, "");
            } break;
            case EventType::Aruco: {
                const auto& x = std::get<ArucoEvent>(ev->payload);
                fusion_.update_with_marker(x.id, x.box, x.ts);
                CSV_LOG_SIMPLE("Ctrl", "IN_ARUCO", (uint64_t)x.id,
                               x.box.x, x.box.y, x.box.width, x.box.height, "");

                // ★ 전환 FSM 입력은 "중기 단계"에서만 평가
                if (GuidanceState::phase().load() == GuidancePhase::Midcourse) {
                    on_aruco_for_transition(x.id, x.box, x.ts);
                }
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
            t_phase_start_ = clock_t::now();

            CSV_LOG_SIMPLE("Ctrl", "SD_REQ", sd->seq, (double)sd->level, 0,0,0, "");
        }
    }
}

void ControlThread::tick_run_mode() {
    CtrlCmd cmd = controller_.solve(fusion_);
    const uint64_t ts = flir::now_ns_steady();

    // ★ 현재 유도 단계 포함하여 로그
    const auto ph = GuidanceState::phase().load();
    CSV_LOG_SIMPLE("Ctrl", "CTRL_OUT",
                   (uint64_t) (ph == GuidancePhase::Midcourse ? 0 : 1),
                   (double)cmd.to_int(), 0,0,0, "");

    // Meta_TxThread (PC)로도 나가게 Topic::Control에 게시 (현행 유지)
    Event ev{ EventType::MetaCtrl, MetaCtrlEvent{ cmd.to_int(), ts } };
    bus_.push(ev, Topic::Control);

    // UART로 비차단 송신 (현행 유지)
    act_.write_nonblock(cmd);
}

void ControlThread::step_shutdown_fsm() {
    if (phase_ == SdPhase::SD_QUIESCE) {
        fusion_.set_quiesce(true);
        act_.set_quiesce(true);

        CtrlCmd sd = CtrlCmd::make_self_destruct();
        const uint64_t ts = flir::now_ns_steady();

        Event ev{ EventType::MetaCtrl, MetaCtrlEvent{ sd.to_int(), ts } };
        bus_.push(ev, Topic::Control);

        act_.write_nonblock(sd);
        CSV_LOG_SIMPLE("Ctrl", "SELF_DESTRUCT_SENT", 0, (double)sd.to_int(), 0,0,0, "");

        phase_ = SdPhase::SD_DONE;
        mode_  = Mode::SHUTDOWN;
    }
}

// ==========================
// ★ 전환 관련 헬퍼
// ==========================
bool ControlThread::is_big_enough(int bw, int bh) const {
    // 절대 기준
    bool big_abs = (bw >= cfg_.guidance.min_bbox_w) &&
                   (bh >= cfg_.guidance.min_bbox_h);

    // (선택) 비율 기준
    bool big_frac = false;
    if (cfg_.guidance.min_bbox_frac > 0.f && cfg_.eo_w > 0 && cfg_.eo_h > 0) {
        float fw = static_cast<float>(bw) / static_cast<float>(cfg_.eo_w);
        float fh = static_cast<float>(bh) / static_cast<float>(cfg_.eo_h);
        big_frac = (std::max(fw, fh) >= cfg_.guidance.min_bbox_frac);
    }
    return big_abs || big_frac;
}

void ControlThread::on_aruco_for_transition(int id, const cv::Rect& box, uint64_t /*ts_ns*/) {
    const auto now = clock_t::now();
    const auto& g  = cfg_.guidance;

    if (id == g.terminal_marker_id) {
        // “충분히 큼” 누적 판정
        if (is_big_enough(box.width, box.height)) {
            big_cnt_ = std::min(big_cnt_ + 1, 1000000);
            if (big_cnt_ >= g.min_big_frames) big_reached_ = true;
        } else {
            big_cnt_ = 0; // 크기 미달이면 누적 리셋
        }
        // 마지막 관측 갱신
        last_seen_tp_ = now;

        CSV_LOG_SIMPLE("Ctrl", "ARU_BIGCHK",
                       (uint64_t)id,
                       (double)box.width, (double)box.height,
                       (double)big_cnt_, (double)big_reached_, "");
    } else {
        // 다른 마커면 크기 누적은 리셋, 단 "마지막 관측"은 갱신하지 않음
        big_cnt_ = 0;
    }

    // “충분히 큼” 이후 일정 시간 미검출 유지 → 전환
    if (big_reached_) {
        const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_seen_tp_).count();
        if (dt_ms >= g.lost_timeout_ms) {
            GuidanceState::phase().store(GuidancePhase::Terminal);

            CSV_LOG_SIMPLE("Ctrl", "PHASE_SWITCH",
                           1 /*Terminal*/, (double)dt_ms, 0,0,0, "");

            // (선택) Meta_TxThread로 전환 통지 이벤트를 보내고 싶다면
            const uint64_t ts = flir::now_ns_steady();
            Event ev_phase{ EventType::MetaCtrl, MetaCtrlEvent{ CtrlCmd::make_phase_switch_terminal().to_int(), ts } };
            bus_.push(ev_phase, Topic::Control);

            // 내부 상태 리셋
            big_cnt_ = 0;
            big_reached_ = false;
        }
    }
}

} // namespace flir

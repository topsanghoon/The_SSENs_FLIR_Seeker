#include "threads_includes/ControlThread.hpp"
#include "util/time_util.hpp"
#include <cassert>
#include <iostream>

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

    // ★ 초기 유도 단계 설정
    GuidanceState::phase().store(cfg_.guidance.default_phase);
    big_cnt_ = 0; big_reached_ = false;
    last_seen_tp_ = clock_t::now();
}

void ControlThread::start() {
    if (running_.exchange(true)) return;

    // 이벤트 기반으로만 깨어난다 (Tracking/Aruco)
    bus_.subscribe(Topic::Tracking, &inbox_, &wake_);
    bus_.subscribe(Topic::Aruco,    &inbox_, &wake_);
    bus_.subscribe(Topic::User,     &inbox_, &wake_);

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
    // 이벤트 버스에 새 이벤트?
    const bool has_evt = (inbox_.latest_seq() > fusion_.last_event_seq());
    // SD 우편함에 새 명령?
    const bool has_sd  = (sd_mb_.latest_seq() > sd_seq_seen_);

    // 틱으로는 절대 깨우지 않음 — 완전 이벤트 드리븐
    return !running_.load() ? true : (has_evt || has_sd);
}

void ControlThread::run() {
    std::cout << "done\n";
    while (running_.load()) {
        {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait(lk, [&]{ return !running_.load() || ready_to_wake(); });
        }
        if (!running_.load()) break;

        handle_self_destruct();          // SD가 오면 즉시 처리
        had_new_sensing_evt_ = drain_events(); // 센싱 이벤트 드레인

        if (mode_ == Mode::RUN) {
            if (had_new_sensing_evt_) {
                // 센싱이 들어왔을 때만 해결/송신
                maybe_emit_control_if_target();
            }
        } else {
            // SHUTDOWN 진행
            std::cout << "come here 2\n";
            step_shutdown_fsm();
        }
    }
}

bool ControlThread::drain_events() {
    bool saw_sensing = false;

    while (auto ev = inbox_.exchange(nullptr)) {
        switch (ev->type) {
            case EventType::Track: {
                const auto& x = std::get<TrackEvent>(ev->payload);
                fusion_.update_with_track(x.box, x.score, x.ts, x.frame_seq);
                controller_.on_track_event();                // ★ 추가
                saw_sensing = true;
                CSV_LOG_SIMPLE("Ctrl", "IN_TRACK", x.frame_seq,
                            x.box.x, x.box.y, x.box.width, x.box.height, "");
            } break;

            case EventType::Aruco: {
                const auto& x = std::get<ArucoEvent>(ev->payload);
                fusion_.update_with_marker(x.id, x.box, x.ts);
                controller_.on_aruco_event(x.id);            // ★ 추가
                saw_sensing = true;
                CSV_LOG_SIMPLE("Ctrl", "IN_ARUCO", (uint64_t)x.id,
                            x.box.x, x.box.y, x.box.width, x.box.height, "");
                if (GuidanceState::phase().load() == GuidancePhase::Midcourse) {
                    on_aruco_for_transition(x.id, x.box, x.ts);
                }
            } break;
            case EventType::SelfDestruct: {
                std::cout << "come here\n";
                const auto& x = std::get<SelfDestructEvent>(ev->payload);
                sd_seq_seen_ = x.seq;
                mode_  = Mode::SHUTDOWN;
                phase_ = SdPhase::SD_QUIESCE;
                CSV_LOG_SIMPLE("Ctrl","SD_REQ",x.seq,(double)x.level,0,0,0,"");
            } break;

            default: break;
        }
        fusion_.bump_last_event_seq();
    }

    return saw_sensing;
}

void ControlThread::handle_self_destruct() {
    if (sd_mb_.has_new(sd_seq_seen_)) {
        if (auto sd = sd_mb_.exchange(nullptr)) {
            sd_seq_seen_ = sd->seq;
            mode_  = Mode::SHUTDOWN;
            phase_ = SdPhase::SD_QUIESCE;

            CSV_LOG_SIMPLE("Ctrl", "SD_REQ", sd->seq, (double)sd->level, 0,0,0, "");
        }
    }
}

void ControlThread::maybe_emit_control_if_target() {
    // 타깃 없으면 송신하지 않음
    auto b = fusion_.last_box();
    if (!has_target_box(b)) {
        CSV_LOG_SIMPLE("Ctrl", "CTRL_SKIP_NO_TARGET", 0, 0,0,0,0, "");
        return;
    }

    // 해결 → 송신
    CtrlCmd cmd = controller_.solve(fusion_);
    const uint64_t ts = flir::now_ns_steady();

    // 현재 유도 단계 포함하여 로그
    const auto ph = GuidanceState::phase().load();
    CSV_LOG_SIMPLE("Ctrl", "CTRL_OUT",
                   (uint64_t) (ph == GuidancePhase::Midcourse ? 0 : 1),
                   (double)cmd.to_int(), 0,0,0, "");

    // Meta_TxThread로 게시
    Event ev{ EventType::MetaCtrl, MetaCtrlEvent{ cmd.to_int(), ts } };
    bus_.push(ev, Topic::Control);

    // UART 비차단 송신
    act_.write_nonblock(cmd);
}

void ControlThread::step_shutdown_fsm() {
    std::cout << "before sd quiesce \n";
    if (phase_ == SdPhase::SD_QUIESCE) {
        std::cout << "after sd quiesce \n";
        fusion_.set_quiesce(true);
        act_.set_quiesce(true);

        CtrlCmd sd = CtrlCmd::make_self_destruct();
        const uint64_t ts = flir::now_ns_steady();

        Event ev{ EventType::MetaCtrl, MetaCtrlEvent{ sd.to_int(), ts } };
        bus_.push(ev, Topic::Control);

        act_.stop_io();
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

    // 터미널 마커만 전환 후보
    if (id == g.terminal_marker_id) {

        if (is_big_enough(box.width, box.height)) {
            // 최초로 "충분히 큼"에 진입한 시각 저장
            if (big_cnt_ == 0) big_reached_tp_ = now;

            // 프레임 누적 (보호 상한)
            big_cnt_ = std::min(big_cnt_ + 1, 1000000);

            // 유지 시간(ms) 계산
            const auto held_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - big_reached_tp_).count();

            // 로그(유지 시간/누적 프레임 확인)
            CSV_LOG_SIMPLE("Ctrl", "ARU_HOLDCHK",
                           (uint64_t)id,
                           (double)box.width, (double)box.height,
                           (double)big_cnt_, (double)held_ms,
                           "");

            // 전환 조건:
            //  - 충분히 큼 상태가 g.min_big_frames 프레임 이상
            //  - 그리고 그 상태가 g.hold_big_ms(ms) 이상 유지
            if (big_cnt_ >= g.min_big_frames && held_ms >= g.hold_big_ms) {
                GuidanceState::phase().store(GuidancePhase::Terminal);

                CSV_LOG_SIMPLE("Ctrl", "PHASE_SWITCH",
                               1 /*Terminal*/, (double)held_ms, (double)big_cnt_, 0,0, "");

                // (선택) Meta_TxThread로 전환 통지
                const uint64_t ts = flir::now_ns_steady();
                Event ev_phase{ EventType::MetaCtrl, MetaCtrlEvent{ CtrlCmd::make_phase_switch_terminal().to_int(), ts } };
                bus_.push(ev_phase, Topic::Control);

                // 내부 상태 리셋
                big_cnt_ = 0;
                big_reached_ = false; // 더 이상 사용하지 않지만 유지 시 false로
                // big_reached_tp_는 유지해도 무방
                return;
            }

        } else {
            // 크기 미달 → 누적/타이머 리셋
            big_cnt_ = 0;
            // big_reached_tp_ = {}; // 필요시 타이머도 완전 리셋
        }

    } else {
        // 다른 마커면 누적 리셋
        big_cnt_ = 0;
    }
}


} // namespace flir

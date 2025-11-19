#include "threads_includes/ControlThread.hpp"
#include "util/time_util.hpp"
#include "util/telemetry.hpp"

#include <cassert>
#include <iostream>
#include <string>
#include <chrono>

namespace flir {

using clock_t = std::chrono::steady_clock;
static constexpr const char* TAG = "Control";

// ─────────────────────────────────────────────
// ★ 이 스레드 전용 루프 컨텍스트 (루프당 1개)
//   - run()에서 만들고, 다른 멤버 함수들은
//     thread_local 포인터를 통해 note에만 접근.
//   - 헤더/멤버 필드 추가 안 해도 됨.
// ─────────────────────────────────────────────
namespace {
struct CtrlLoopContext {
    std::uint64_t seq{0};
    std::uint64_t t0{0}, t1{0}, t2{0}, t3{0};
    std::string   note;
};

thread_local CtrlLoopContext* g_ctrl_loop_ctx = nullptr;

inline void append_note(const std::string& s) {
    if (!g_ctrl_loop_ctx) return;
    if (!g_ctrl_loop_ctx->note.empty())
        g_ctrl_loop_ctx->note += ';';
    g_ctrl_loop_ctx->note += s;
}
} // anon namespace

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

    big_cnt_ = 0;
    big_reached_ = false;
    last_seen_tp_  = clock_t::now();

    // ★ 타임아웃 기준/상태 초기화
    last_ctrl_tp_            = clock_t::now();
    neutral_sent_after_loss_ = false;
    use_timeout_wait_        = true;   // 처음에는 타임아웃 감시 ON
}

void ControlThread::start() {
    if (running_.exchange(true)) return;

    // 이벤트 기반으로만 깨어난다 (Tracking/Aruco/User)
    bus_.subscribe(Topic::Tracking, &inbox_, &wake_);
    bus_.subscribe(Topic::Aruco,    &inbox_, &wake_);
    bus_.subscribe(Topic::User,     &inbox_, &wake_);

    th_ = std::thread(&ControlThread::run, this);

    // ★ 스레드 시작 이벤트 (예외적으로 별도 1줄)
    CSV_LOG_TL("Ctrl",
               0,     // seq
               0,0,0,0,
               0,     // total 자동계산 안 함(0)
               "THREAD_START");

    // ★ 프로그램 시작 직후 UART로 start 신호 전송
    act_.send_start_signal();

    // ★ 프로그램 시작 메타 신호 전송 (예: 8001)
    const uint64_t ts = flir::now_ns_steady();
    CtrlCmd start_cmd = CtrlCmd::make_start_signal();
    Event ev_start{ EventType::MetaCtrl, MetaCtrlEvent{ start_cmd.to_int(), ts } };
    bus_.push(ev_start, Topic::Control);
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

    // ★ 스레드 종료 이벤트 (예외적으로 별도 1줄)
    CSV_LOG_TL("Ctrl",
               0,     // seq
               0,0,0,0,
               0,
               "THREAD_STOP");
}

bool ControlThread::ready_to_wake() {
    // 이벤트 버스에 새 이벤트?
    const bool has_evt = (inbox_.latest_seq() > fusion_.last_event_seq());
    // SD 우편함에 새 명령?
    const bool has_sd  = (sd_mb_.latest_seq() > sd_seq_seen_);

    // 틱으로는 절대 깨우지 않음 — (타임아웃 모드일 때만 wait_for 사용)
    return !running_.load() ? true : (has_evt || has_sd);
}

void ControlThread::run() {
    std::cout << "done\n";

    static std::uint64_t seq_counter = 0;

    while (running_.load()) {
        bool woke_by_evt = false;
        {
            std::unique_lock<std::mutex> lk(m_);

            if (use_timeout_wait_) {
                // ★ 타임아웃 감시 ON인 동안에는 최대 20ms마다 한 번씩 깨어나서
                //   "마지막 제어 이후 0.3초 경과"를 검사할 수 있게 해준다.
                woke_by_evt = cv_.wait_for(
                    lk,
                    std::chrono::milliseconds(20),
                    [&]{ return !running_.load() || ready_to_wake(); }
                );
            } else {
                // ★ 타임아웃 감시 OFF → 순수 이벤트 드리븐 모드
                cv_.wait(lk, [&]{ return !running_.load() || ready_to_wake(); });
                woke_by_evt = true; // 여기서 깬 건 이벤트 때문으로 취급
            }
        }
        if (!running_.load()) break;

        CtrlLoopContext ctx{};
        ctx.seq = ++seq_counter;
        g_ctrl_loop_ctx = &ctx;

        // T0: 깨어난 직후
        ctx.t0 = flir::now_us_steady();

        if (woke_by_evt) {
            // SD 우편함 체크
            handle_self_destruct();
            ctx.t1 = flir::now_us_steady();

            // 이벤트 버스로부터 센싱/SD 이벤트 드레인
            had_new_sensing_evt_ = drain_events();
            ctx.t2 = flir::now_us_steady();

            if (mode_ == Mode::RUN) {
                if (had_new_sensing_evt_) {
                    // 센싱이 들어왔을 때만 해결/송신
                    maybe_emit_control_if_target();
                } else {
                    append_note("NO_SENSING_EVT");
                }
            } else {
                // SHUTDOWN 진행
                step_shutdown_fsm();
            }
        } else {
            // 타임아웃으로만 깨어난 경우 (이벤트 없음)
            ctx.t1 = ctx.t0;
            ctx.t2 = ctx.t0;
            append_note("TIMEOUT_WAKE");
        }

        // ★ 여기서 “마지막 제어 이후 0.3초 경과” 검사 후 중립(0) 명령 전송
        if (mode_ == Mode::RUN && use_timeout_wait_) {
            auto now_tp  = clock_t::now();
            auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now_tp - last_ctrl_tp_).count();

            if (dt_ms >= 300 && !neutral_sent_after_loss_) {
                // 중립 명령 생성 (p1 = 0 → 직진)
                CtrlCmd neutral{};
                neutral.p1 = 0;

                const uint64_t ts = flir::now_ns_steady();

                append_note("CTRL_NEUTRAL_TIMEOUT");

                // 메타 채널로도 보내고 싶으면 유지
                Event ev{ EventType::MetaCtrl, MetaCtrlEvent{ 0, ts } };
                bus_.push(ev, Topic::Control);

                // UART로 0 전송
                act_.write_nonblock(neutral);

                // 상태 갱신: 같은 loss 에피소드에서 한 번만 보내기
                neutral_sent_after_loss_ = true;
                last_ctrl_tp_            = now_tp;

                // ★ 이번 에피소드에서는 중립을 이미 보냈으므로
                //    더 이상 주기적으로 타임아웃 확인할 필요 없음
                use_timeout_wait_ = false;
            }
        }

        // T3: 전체 루프 끝
        ctx.t3 = flir::now_us_steady();

        if (ctx.note.empty()) {
            ctx.note = woke_by_evt ? "IDLE_WAKE" : "TIMEOUT_ONLY";
        }

        CSV_LOG_TL("Ctrl",
                   ctx.seq,
                   ctx.t0, ctx.t1, ctx.t2, ctx.t3,
                   0,
                   ctx.note);

        g_ctrl_loop_ctx = nullptr;
    }
}

bool ControlThread::drain_events() {
    bool saw_sensing = false;

    while (auto ev = inbox_.exchange(nullptr)) {
        switch (ev->type) {
            case EventType::Track: {
                const auto& x = std::get<TrackEvent>(ev->payload);
                fusion_.update_with_track(x.box, x.score, x.ts, x.frame_seq);
                controller_.on_track_event();
                saw_sensing = true;

                append_note("IN_TRACK:frame=" + std::to_string(x.frame_seq));
            } break;

            case EventType::Aruco: {
                const auto& x = std::get<ArucoEvent>(ev->payload);
                fusion_.update_with_marker(x.id, x.box, x.ts);
                controller_.on_aruco_event(x.id);
                saw_sensing = true;

                append_note("IN_ARUCO:id=" + std::to_string(x.id));

                if (GuidanceState::phase().load() == GuidancePhase::Midcourse) {
                    on_aruco_for_transition(x.id, x.box, x.ts);
                }
            } break;

            case EventType::SelfDestruct: {
                const auto& x = std::get<SelfDestructEvent>(ev->payload);
                sd_seq_seen_ = x.seq;
                mode_  = Mode::SHUTDOWN;
                phase_ = SdPhase::SD_QUIESCE;

                append_note("SD_REQ_EVT:seq=" +
                            std::to_string(x.seq) +
                            ",level=" + std::to_string(x.level));
            } break;

            default:
                break;
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

            append_note("SD_REQ_MB:seq=" +
                        std::to_string(sd->seq) +
                        ",level=" + std::to_string(sd->level));
        }
    }
}

void ControlThread::maybe_emit_control_if_target() {
    // 타깃 없으면 송신하지 않음
    auto b = fusion_.last_box();
    if (!has_target_box(b)) {
        append_note("CTRL_SKIP_NO_TARGET");
        return;
    }

    // 해결 → 송신
    CtrlCmd cmd = controller_.solve(fusion_);
    const uint64_t ts = flir::now_ns_steady();

    const auto ph = GuidanceState::phase().load();
    const char* ph_str =
        (ph == GuidancePhase::Midcourse ? "Midcourse" : "Terminal");

    append_note(std::string("CTRL_OUT:phase=") +
                ph_str +
                ",cmd=" + std::to_string(cmd.to_int()));

    // Meta_TxThread로 게시 (p1 값만 전송)
    Event ev{ EventType::MetaCtrl, MetaCtrlEvent{ (int)cmd.p1, ts } };
    bus_.push(ev, Topic::Control);

    // UART 비차단 송신
    act_.write_nonblock(cmd);

    // ★ 마지막 제어 시간 / 중립 플래그 갱신
    last_ctrl_tp_            = clock_t::now();
    neutral_sent_after_loss_ = false;

    // ★ 새 제어가 나왔으므로 다시 타임아웃 감시 ON
    use_timeout_wait_        = true;
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

        append_note("SELF_DESTRUCT_SENT:cmd=" + std::to_string(sd.to_int()));

        phase_ = SdPhase::SD_DONE;
        mode_  = Mode::SHUTDOWN;

        if (on_shutdown_) on_shutdown_();
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

void ControlThread::on_aruco_for_transition(int id,
                                            const cv::Rect& box,
                                            uint64_t /*ts_ns*/) {
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
            const auto held_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - big_reached_tp_).count();

            append_note("ARU_HOLDCHK:id=" + std::to_string(id) +
                        ",w=" + std::to_string(box.width) +
                        ",h=" + std::to_string(box.height) +
                        ",cnt=" + std::to_string(big_cnt_) +
                        ",held_ms=" + std::to_string(held_ms));

            // 전환 조건:
            //  - 충분히 큼 상태가 g.min_big_frames 프레임 이상
            //  - 그리고 그 상태가 g.hold_big_ms(ms) 이상 유지
            if (big_cnt_ >= g.min_big_frames && held_ms >= g.hold_big_ms) {
                GuidanceState::phase().store(GuidancePhase::Terminal);

                append_note("PHASE_SWITCH:Terminal");

                // (선택) Meta_TxThread로 전환 통지
                const uint64_t ts = flir::now_ns_steady();
                Event ev_phase{ EventType::MetaCtrl,
                                MetaCtrlEvent{
                                    CtrlCmd::make_phase_switch_terminal().to_int(),
                                    ts } };
                bus_.push(ev_phase, Topic::Control);

                // 내부 상태 리셋
                big_cnt_ = 0;
                big_reached_ = false;
                return;
            }

        } else {
            // 크기 미달 → 누적/타이머 리셋
            big_cnt_ = 0;
        }

    } else {
        // 다른 마커면 누적 리셋
        big_cnt_ = 0;
    }
}

} // namespace flir

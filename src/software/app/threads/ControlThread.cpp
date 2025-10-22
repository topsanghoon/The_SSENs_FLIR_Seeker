#include "threads_includes/ControlThread.hpp"
#include <cassert>

namespace flir {

// =====================================================
// ControlThread: 전체 제어 통합 스레드
// -----------------------------------------------------
// - Tracking / ArUco 스레드로부터 EventBus를 통해 이벤트를 받음
// - TargetFusion 객체에 최신 타겟 정보 갱신
// - Controller를 통해 좌/우/중앙 명령 계산
// - UART 포트로 제어명령 송신
// - MetaCtrlEvent를 EventBus로 발행 (MetaTxThread가 UDP로 송신)
// - SelfDestructCmd(자폭 명령)를 받으면 안전 정지 시퀀스 수행
// =====================================================

using clock_t = std::chrono::steady_clock;

// ----------------------------
// 기본 생성자: Config 기본값 사용 (위임)
// ----------------------------
ControlThread::ControlThread(IEventBus& bus,
                             SpscMailbox<SelfDestructCmd>& sd_mb,
                             TargetFusion& fusion,
                             IController& controller,
                             IActuatorPort& act,
                             CsvLoggerCtrl& logger)
: ControlThread(bus, sd_mb, fusion, controller, act, logger, Config{}) {} // 위임

// ----------------------------
// 전체 구성요소 초기화
// ----------------------------
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
    // Condition Variable + Mutex 초기화
    wake_.cv = &cv_;
    wake_.mu = &m_;
    // 다음 tick 시간 예약 (주기성 제어)
    next_tick_tp_ = clock_t::now() + std::chrono::milliseconds(cfg_.period_ms);
}

// =====================================================
// start(): 스레드 시작
// -----------------------------------------------------
// - EventBus 구독 시작 (Tracking, ArUco)
// - 별도 스레드에서 run() 실행
// =====================================================
void ControlThread::start() {
    running_.store(true);
    bus_.subscribe(Topic::Tracking, &inbox_, &wake_);
    bus_.subscribe(Topic::Aruco,    &inbox_, &wake_);
    th_ = std::thread(&ControlThread::run, this);
}

// =====================================================
// stop(): 실행 중지 요청
// =====================================================
void ControlThread::stop() {
    running_.store(false);
    {
        std::lock_guard<std::mutex> lk(m_);
        cv_.notify_all(); // run() 깨우기
    }
}

// =====================================================
// join(): 스레드 종료 대기 및 구독 해제
// =====================================================
void ControlThread::join() {
    if (th_.joinable()) th_.join();
    bus_.unsubscribe(&inbox_);
}

// =====================================================
// ready_to_wake(): 다음 tick or 새 이벤트/SD명령 도착 여부 확인
// =====================================================
bool ControlThread::ready_to_wake() {
    const bool has_evt = (inbox_.latest_seq() > fusion_.last_event_seq());
    const bool has_sd  = (sd_mb_.latest_seq() > sd_seq_seen_);
    const bool tick    = (clock_t::now() >= next_tick_tp_);
    return !running_.load() ? true : (has_evt || has_sd || tick);
}

// =====================================================
// run(): 주기 루프
// -----------------------------------------------------
// - 매 tick 주기로 Control Thread 주기적 실행
// - 새 이벤트/SD명령이 없으면 대기
// =====================================================
void ControlThread::run() {
    while (running_.load()) {
        {
            std::unique_lock<std::mutex> lk(m_);
            // tick or wake 신호까지 대기
            cv_.wait_until(lk, next_tick_tp_, [&]{ return ready_to_wake(); });
        }
        if (!running_.load()) break;

        // ① 자폭 명령 확인 (안전 정지 단계 진입 여부)
        handle_self_destruct();
        // ② Tracking/Aruco 이벤트 처리 및 TargetFusion 갱신
        drain_events();
        // ③ 동작 모드에 따라 제어 혹은 정지 FSM 수행
        if (mode_ == Mode::RUN) {
            tick_run_mode();
        } else {
            step_shutdown_fsm();
        }

        // 다음 주기 예약
        next_tick_tp_ = clock_t::now() + std::chrono::milliseconds(cfg_.period_ms);
    }
}

// =====================================================
// drain_events(): 이벤트 수신 → TargetFusion 반영
// =====================================================
void ControlThread::drain_events() {
    while (auto ev = inbox_.exchange(nullptr)) {
        switch (ev->type) {
            case EventType::Track: {
                // IR Tracking 결과 업데이트
                const auto& x = std::get<TrackEvent>(ev->payload);
                fusion_.update_with_track(x.box, x.score, x.ts, x.frame_seq);
            } break;
            case EventType::Aruco: {
                // ArUco 마커 감지 결과 업데이트
                const auto& x = std::get<ArucoEvent>(ev->payload);
                fusion_.update_with_marker(x.id, x.box, x.ts);
            } break;
            default: break;
        }
        // 마지막 이벤트 시퀀스 갱신
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
            phase_ = SdPhase::SD_QUIESCE;          // 첫 단계 진입
            t_phase_start_ = clock_t::now();
            log_.sd_req(sd->seq, sd->level);
        }
    }
}

// =====================================================
// tick_run_mode(): 정상 제어 주기
// -----------------------------------------------------
// - Controller에서 CtrlCmd 계산
// - MetaCtrlEvent를 EventBus에 게시 (MetaTxThread로 전달됨)
// - Actuator(UART)로 명령 전송
// - CSV 로그 기록
// =====================================================
void ControlThread::tick_run_mode() {
    // 1) 제어 계산
    CtrlCmd cmd = controller_.solve(fusion_);
    // 2) EventBus 게시 (다른 스레드로 전달됨)
    Event ev{ EventType::MetaCtrl, MetaCtrlEvent{ cmd.to_int(), controller_.now_ns() } };
    bus_.push(ev, Topic::Control);
    // 3) UART 포트로 명령 송신
    act_.write_nonblock(cmd);
    // 4) 로그 기록
    log_.ctrl_out(cmd);
}

// =====================================================
// step_shutdown_fsm(): 안전 정지 FSM 단계별 수행
// -----------------------------------------------------
//  [QUIESCE] → [PARK] → [STOP_IO] → [DONE]
// =====================================================
void ControlThread::step_shutdown_fsm() {
    using std::chrono::milliseconds;
    auto elapsed = std::chrono::duration_cast<milliseconds>(clock_t::now() - t_phase_start_).count();

    switch (phase_) {
        case SdPhase::SD_QUIESCE: {
            // (1) 더 이상 새로운 데이터 수신하지 않음
            fusion_.set_quiesce(true);
            act_.set_quiesce(true);
            log_.phase("QUIESCE");
            if (elapsed > cfg_.sd_quiesce_ms) {
                phase_ = SdPhase::SD_PARK; t_phase_start_ = clock_t::now();
            }
        } break;

        case SdPhase::SD_PARK: {
            // (2) 안전 자세로 이동 (예: 방향 0)
            CtrlCmd safe = CtrlCmd::safe_pose();
            act_.write_nonblock(safe);
            log_.phase("PARK");
            if (elapsed > cfg_.sd_park_ms) {
                phase_ = SdPhase::SD_STOP_IO; t_phase_start_ = clock_t::now();
            }
        } break;

        case SdPhase::SD_STOP_IO: {
            // (3) 통신 정지
            act_.stop_io();
            fusion_.drain_for_ms(100);
            log_.phase("STOP_IO");
            phase_ = SdPhase::SD_DONE;
        } break;

        case SdPhase::SD_DONE: {
            // (4) 완전 정지 완료
            log_.sd_done();
        } break;
        default: break;
    }
}

} // namespace flir

#pragma once
#include <atomic>
#include <thread>
#include <opencv2/core.hpp>

#include "ipc/ipc_types.hpp"   // Event/UserCmd/FrameHandle 등
#include "ipc/mailbox.hpp"     // SpscMailbox<T>
#include "ipc/event_bus.hpp"   // IEventBus
#include "components/includes/IR_Frame.hpp"

namespace flir {

// ---------- 전략/정책/로거 ----------
struct IPreprocessor {
    virtual ~IPreprocessor() = default;
    // 호출자: IR_TrackThread (init/update 직전)
    virtual void run(const IRFrame16& in16, cv::Mat& out32f) = 0; // GRAY16 → GRAY32F
};

struct ITrackerStrategy {
    virtual ~ITrackerStrategy() = default;
    // 호출자: IR_TrackThread (새 타깃 지정 직후)
    virtual bool init(const cv::Mat& pf, const cv::Rect2f& box) = 0;
    // 호출자: IR_TrackThread (각 프레임)
    virtual bool update(const cv::Mat& pf, cv::Rect2f& out_box, float& score) = 0;
};

struct IReinitHintPolicy {
    virtual ~IReinitHintPolicy() = default;
    // 호출자: IR_TrackThread (연속 실패 == 3회 시)
    virtual cv::Rect2f suggest(const cv::Rect2f& last_box) = 0;
};

struct CsvLogger {
    // 호출자: IR_TrackThread (각 시점 로깅)
    void click(uint32_t click_id, const cv::Rect2f& box);
    void init_ok(uint32_t frame_id, double ms);
    void init_fail(int fail_streak, double ms);
    void track_ok(uint32_t frame_id, float score, double ms);
    void track_lost(int fail_streak, double ms);
};

struct IRTrackConfig {
    int  reinit_threshold   = 3; // 3회 실패 → 다음 프레임에 재초기화 시도
    int  user_req_threshold = 9; // 9회 실패 → 사용자 재지정 요구
};

// ---------- 스레드 본체 ----------
class IR_TrackThread {
public:
    IR_TrackThread(SpscMailbox<IRFrameHandle>& ir_mb,
                   SpscMailbox<UserCmd>&     click_mb,
                   ITrackerStrategy&         tracker,
                   IPreprocessor&            preproc,
                   IReinitHintPolicy&        reinit,
                   IEventBus&                bus,
                   CsvLogger                 logger,
                   IRTrackConfig             cfg = {});

    // 수명 제어: main/ThreadManager에서 호출
    void start();   // 내부에서 std::thread 생성, run() 실행
    void stop();    // 종료 플래그 set + 대기 해제
    void join();    // 외부에서 합류

    // 생산자(선택 경로): 직접 주입 + 깨움
    void onClickArrived(const UserCmd& cmd);
    void onFrameArrived(IRFrameHandle h);

private:
    // === 협력자 / 구성 ===
    SpscMailbox<IRFrameHandle>& ir_mb_;
    SpscMailbox<UserCmd>&     click_mb_;
    ITrackerStrategy&         tracker_;
    IPreprocessor&            preproc_;
    IReinitHintPolicy&        reinit_;
    IEventBus&                bus_;
    CsvLogger                 log_;
    IRTrackConfig             cfg_;

    // === 스레드 관리 ===
    std::thread        th_;
    std::atomic<bool>  running_{false};

    // === 추적 상태 ===
    bool       new_target_     = false;
    bool       tracking_valid_ = false;
    cv::Rect2f target_box_{};
    int        fail_streak_    = 0;
    uint32_t   frame_seq_seen_ = 0;
    uint32_t   click_seq_seen_ = 0;

    // === 내부 루프/헬퍼 ===
    void run();                  // 워커 스레드 본체(비공개)
    void wait_until_ready();     // CLICK/FRAME 없으면 대기
    void handle_click(const UserCmd& cmd);
    void on_frame(IRFrameHandle& h);
    bool try_init(const cv::Mat& pf, const cv::Rect2f& box);
    bool try_update(const cv::Mat& pf, cv::Rect2f& out_box, float& score);
    void emit_init(const cv::Rect2f& b, uint64_t ts);
    void emit_track(const cv::Rect2f& b, float score, uint64_t ts);
    void emit_lost(const cv::Rect2f& last, uint64_t ts);
    void emit_need_reselect();
};

} // namespace flir

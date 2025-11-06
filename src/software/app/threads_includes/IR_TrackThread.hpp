// IR_TrackThread.hpp  (CSV unified / CsvLoggerIR 제거)
#pragma once

#include <atomic>
#include <thread>
#include <memory>
#include <opencv2/core.hpp>
#include <condition_variable>

#include "ipc/ipc_types.hpp"            // Event, EventType, TrackEvent, ...
#include "ipc/mailbox.hpp"              // SpscMailbox<>
#include "ipc/event_bus.hpp"            // IEventBus
#include "components/includes/IR_Frame.hpp"

// ✅ 공통 로거/타임/플래그
#include "util/common_log.hpp"      // LOGD/LOGI/LOGW/LOGE
#include "util/time_util.hpp"       // flir::ScopedTimerMs
#include "util/telemetry.hpp"       // FLIR_* 플래그

namespace flir {

// 전방 선언(구체 구현은 바깥에 있음)
class ITrackerStrategy {
public:
    virtual ~ITrackerStrategy() = default;
    virtual bool init(const cv::Mat& pf32, const cv::Rect2f& box) = 0;
    virtual bool update(const cv::Mat& pf32, cv::Rect2f& out_box, float& score) = 0;
};

class IPreprocessor {
public:
    virtual ~IPreprocessor() = default;

    // 16-bit RAW 입력을 32F로 변환
    virtual void run(const IRFrame16& in, cv::Mat& out_pf32) = 0;
};

// 트래커 동작 관련 간단 설정(필요 항목만 남김)
struct IRTrackConfig {
    int  user_req_threshold{5};   // 잃어버림 N회 시 NeedReselect 알림
};

class IR_TrackThread {
public:
    IR_TrackThread(SpscMailbox<std::shared_ptr<IRFrameHandle>>& ir_mb,
                   SpscMailbox<UserCmd>&     click_mb,
                   ITrackerStrategy&         tracker,
                   IPreprocessor&            preproc,
                   IEventBus&                bus,
                   IRTrackConfig             cfg);

    void start();
    void stop();
    void join();

    // 외부 스레드가 이벤트를 밀어 넣어 깨우는 진입점
    void onFrameArrived(std::shared_ptr<IRFrameHandle> h);
    void onClickArrived(const UserCmd& cmd);

    std::unique_ptr<WakeHandle> create_wake_handle();   // ★ 추가

private:
    void run();
    void wait_until_ready();

    void handle_click(const UserCmd& cmd);
    void on_frame(IRFrameHandle& h);

    bool try_init(const cv::Mat& pf, const cv::Rect2f& box);
    bool try_update(const cv::Mat& pf, cv::Rect2f& out_box, float& score);

    void emit_init(const cv::Rect2f& b, uint64_t ts);
    void emit_track(const cv::Rect2f& b, float score, uint64_t ts);
    void emit_lost(const cv::Rect2f& last, uint64_t ts);
    void emit_need_reselect();

    void cleanup();


    // 의존성
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& ir_mb_;
    SpscMailbox<UserCmd>&     click_mb_;
    ITrackerStrategy&         tracker_;
    IPreprocessor&            preproc_;
    IEventBus&                bus_;
    IRTrackConfig             cfg_;

    // 스레드
    std::thread       th_;
    std::atomic<bool> running_{false};

 
    uint32_t ir_mb_seq_seen_{0};
    uint32_t click_mb_seq_seen_{0};
    uint32_t last_frame_seq_{0};

    // 상태
    cv::Rect2f target_box_{};
    bool       new_target_{false};
    bool       tracking_valid_{false};
    int        fail_streak_{0};
    bool       reselect_notified_{false};

    std::condition_variable cv_;   // ★ 추가
    std::mutex              m_;    // ★ 추가
};

} // namespace flir

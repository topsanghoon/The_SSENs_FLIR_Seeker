#pragma once
#include <atomic>
#include <thread>
#include <vector>
#include <opencv2/core.hpp>

#include "ipc/ipc_types.hpp"   // Event/UserCmd/FrameHandle 등
#include "ipc/mailbox.hpp"     // SpscMailbox<T>
#include "ipc/event_bus.hpp"   // IEventBus

namespace flir {

// 전처리: EO 프레임 → ArUco 입력(grayscale 등)
struct IArucoPreprocessor {
    virtual ~IArucoPreprocessor() = default;
    // 호출자: EO_ArUcoThread (검출 직전)
    // in: FrameHandle(EO 원본), out: pf(검출용 그레이/필터 결과)
    virtual void run(const EOFrameHandle& in, cv::Mat& pf_gray8) = 0;
};

// 검출기: 전처리 결과에서 마커 검출
struct IArucoDetector {
    virtual ~IArucoDetector() = default;
    struct Detection { int id; cv::Rect2f bbox; /* pose 등 필요시 추가 */ };
    // 호출자: EO_ArUcoThread
    virtual std::vector<Detection> detect(const cv::Mat& pf_gray8) = 0;
};

// 로깅
struct CsvLoggerAru {
    void marker_found(uint32_t frame_id, size_t count, double ms); // EO_ArUcoThread_log.csv
    void marker_lost (uint32_t frame_id, double ms);
};

class EO_ArUcoThread {
public:
    EO_ArUcoThread(SpscMailbox<EOFrameHandle>& eo_mb,
                   IArucoPreprocessor&       preproc,
                   IArucoDetector&           detector,
                   IEventBus&                bus,
                   CsvLoggerAru              logger);

    // 수명 제어
    void start();
    void stop();
    void join();

    // 생산자(EO_CaptureThread)에서 직접 깨우고 싶을 때 사용 가능(선택)
    void onFrameArrived(EOFrameHandle h);

private:
    // 협력자
    SpscMailbox<EOFrameHandle>& eo_mb_;
    IArucoPreprocessor&       preproc_;
    IArucoDetector&           detector_;
    IEventBus&                bus_;
    CsvLoggerAru              log_;

    // 상태
    std::thread              th_;
    std::atomic<bool>        running_{false};
    uint32_t                 frame_seq_seen_ = 0;

    // 내부 루프/헬퍼
    void run();                  // 워커 스레드 본체
    void wait_until_ready();     // 새 EO 프레임 없으면 대기
    void on_frame(EOFrameHandle& h);

    // 이벤트 발행
    void emit_aruco(int id, const cv::Rect2f& box, uint64_t ts_ns, uint32_t frame_seq);
};

} // namespace flir

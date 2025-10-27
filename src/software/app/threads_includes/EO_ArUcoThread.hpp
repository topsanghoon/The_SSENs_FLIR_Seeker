// threads_includes/EO_ArUcoThread.hpp
#pragma once
#include <atomic>
#include <thread>
#include <vector>
#include <array>
#include <opencv2/core.hpp>
#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/event_bus.hpp"
#include "components/includes/EO_Frame.hpp"

namespace flir {

struct IArucoPreprocessor {
    virtual ~IArucoPreprocessor() = default;
    virtual void run(const EOFrameHandle& in, cv::Mat& pf_gray8) = 0;
};

struct IArucoDetector {
    virtual ~IArucoDetector() = default;
    struct Detection {
        int id;
        std::array<cv::Point2f,4> corners;
        cv::Rect2f bbox;
    };
    virtual std::vector<Detection> detect(const cv::Mat& pf_gray8) = 0;
};

class CsvLoggerAru; // 전방선언 (구체 타입)

class EO_ArUcoThread {
public:
    EO_ArUcoThread(SpscMailbox<std::shared_ptr<EOFrameHandle>>& eo_mb,
                   IArucoPreprocessor&       preproc,
                   IArucoDetector&           detector,
                   IEventBus&                bus,
                   CsvLoggerAru&             logger);   // ← 구체 타입 참조

    void start();
    void stop();
    void join();
    void onFrameArrived(std::shared_ptr<EOFrameHandle> h);

private:
    SpscMailbox<std::shared_ptr<EOFrameHandle>>& eo_mb_;
    IArucoPreprocessor&       preproc_;
    IArucoDetector&           detector_;
    IEventBus&                bus_;
    CsvLoggerAru&             log_;     // ← 구체 타입 참조

    std::thread              th_;
    std::atomic<bool>        running_{false};
    uint32_t                 frame_seq_seen_ = 0;

    void run();
    void wait_until_ready();
    void on_frame(const std::shared_ptr<EOFrameHandle>& h);
    void emit_aruco(int id,
                    const std::array<cv::Point2f,4>& corners,
                    const cv::Rect2f& box,
                    uint64_t ts_ns, uint32_t frame_seq);
    void log_debug(const std::string& msg);
};

} // namespace flir

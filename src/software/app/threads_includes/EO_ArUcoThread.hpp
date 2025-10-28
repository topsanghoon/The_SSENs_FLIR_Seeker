#pragma once
#include <atomic>
#include <thread>
#include <memory>
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
    struct Detection {
        int id;
        cv::Rect2f bbox;
        std::array<cv::Point2f,4> corners;
    };
    virtual ~IArucoDetector() = default;
    virtual std::vector<Detection> detect(const cv::Mat& gray8) = 0;
};

class EO_ArUcoThread {
public:
    EO_ArUcoThread(SpscMailbox<std::shared_ptr<EOFrameHandle>>& eo_mb,
                   IArucoPreprocessor& preproc,
                   IArucoDetector& detector,
                   IEventBus& bus);

    void start();
    void stop();
    void join();

    void onFrameArrived(std::shared_ptr<EOFrameHandle> h);

private:
    void run();
    void wait_until_ready();
    void on_frame(const std::shared_ptr<EOFrameHandle>& h);

    void emit_aruco(int id,
                    const std::array<cv::Point2f,4>& corners,
                    const cv::Rect2f& box,
                    uint64_t ts_ns, uint32_t frame_seq);

private:
    SpscMailbox<std::shared_ptr<EOFrameHandle>>& eo_mb_;
    IArucoPreprocessor& preproc_;
    IArucoDetector& detector_;
    IEventBus& bus_;

    std::thread th_;
    std::atomic<bool> running_{false};
    uint32_t frame_seq_seen_{0};
};

} // namespace flir

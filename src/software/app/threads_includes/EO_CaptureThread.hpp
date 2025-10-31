// threads_includes/EO_CaptureThread.hpp
#pragma once
#include <atomic>
#include <thread>
#include <memory>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "components/includes/EO_Frame.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/wake.hpp"
#include "main_config.hpp"   // AppConfigPtr
#include "phase_gate.hpp"    // eo_enabled()

namespace flir {

struct EOMatHandle : EOFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    FrameBGR8 owned{};
    EOMatHandle(){ p = &owned; }
    void retain() override {}
    void release() override { keep.reset(); }
};

class EO_CaptureThread {
public:
    EO_CaptureThread(std::string name,
                     SpscMailbox<std::shared_ptr<EOFrameHandle>>& output_mb,
                     std::unique_ptr<WakeHandle> wake,
                     AppConfigPtr app);
    ~EO_CaptureThread();

    void start();
    void stop();
    void join();

private:
    std::string name_;
    SpscMailbox<std::shared_ptr<EOFrameHandle>>& out_;
    std::unique_ptr<WakeHandle> wake_;
    AppConfigPtr app_;

    std::thread th_;
    std::atomic<bool> running_{false};
    cv::VideoCapture cap_;

    void run();
    bool init_cam();
    void close_cam();
    std::shared_ptr<EOFrameHandle> make_handle(const cv::Mat& bgr);
};

} // namespace flir

// threads_includes/IR_CaptureThread.hpp
#pragma once
#include <atomic>
#include <thread>
#include <memory>
#include <vector>
#include <chrono>
#include <opencv2/core.hpp>

#include "components/includes/IR_Frame.hpp"
#include "ipc/mailbox.hpp"
#include "main_config.hpp"    // AppConfigPtr
#include "phase_gate.hpp"     // ir_enabled()

namespace flir {

struct IRMatHandle : IRFrameHandle {
    std::shared_ptr<cv::Mat> keep;
    IRFrame16 owned{};
    IRMatHandle(){ p = &owned; }
    void retain() override {}
    void release() override { keep.reset(); }
};

class IR_CaptureThread {
public:
    IR_CaptureThread(std::string name,
                     SpscMailbox<std::shared_ptr<IRFrameHandle>>& output_mb,
                     AppConfigPtr app);
    ~IR_CaptureThread();

    void start();
    void stop();
    void join();

    uint64_t frames()  const { return frame_count_.load(); }
    uint64_t errors()  const { return error_count_.load(); }

private:
    std::string name_;
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& out_;
    AppConfigPtr app_;

    std::thread th_;
    std::atomic<bool> running_{false};
    int spi_fd_{-1};

    std::vector<uint16_t> frame_buf_;  // W*H
    std::vector<uint8_t>  pkt_;        // 164 bytes (VoSPI packet)

    std::atomic<uint64_t> frame_count_{0};
    std::atomic<uint64_t> error_count_{0};

    void run();
    bool init_spi();
    void close_spi();

    // 2.5/3.0 자동 분기
    bool capture_frame_any();
    bool capture_frame_25();   // 80x60
    bool capture_frame_30();   // 160x120

    bool read_packet(uint8_t* buf, size_t len);
    std::shared_ptr<IRFrameHandle> make_handle();
};

} // namespace flir

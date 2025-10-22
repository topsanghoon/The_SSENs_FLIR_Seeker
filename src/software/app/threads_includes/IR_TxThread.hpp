#pragma once

#include <thread>
#include <atomic>
#include <string>
#include <mutex>
#include <condition_variable>
#include <memory>
// #include <gst/gst.h>
// #include <gst/app/gstappsrc.h>

#include "components/includes/IR_Frame.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/wake.hpp"

// GStreamer 타입 전방 선언
struct _GstElement;
struct _GstAppSrc;

namespace flir {

class IR_TxThread {
public:
    struct GstConfig {
        std::string pc_ip = "192.168.0.179"; // 데이터를 수신할 PC의 IP 주소
        int         port = 5000;             // 사용할 UDP 포트
        int         width = 160;             // 영상 너비
        int         height = 120;            // 영상 높이
        int         fps = 9;                 // Lepton 3의 프레임 속도
    };

    // Constructor with custom GStreamer configuration
    IR_TxThread(
        std::string name,
        SpscMailbox<std::shared_ptr<IRFrameHandle>>& mb,
        const GstConfig& gst_config
    );
    
    // Constructor with default GStreamer configuration
    IR_TxThread(
        std::string name,
        SpscMailbox<std::shared_ptr<IRFrameHandle>>& mb
    );
    
    ~IR_TxThread();

    void start();
    void stop();
    void join();
    
    std::unique_ptr<WakeHandle> create_wake_handle();

private:
    void run();
    void wait_for_frame();
    bool initialize_gstreamer();
    void push_frame_to_gst(const std::shared_ptr<IRFrameHandle>& handle);

    // === 협력자 / 구성 ===
    std::string name_;
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& mb_; 
    const GstConfig gst_config_;
    
    // === 스레드 관리 ===
    std::thread th_;
    std::atomic<bool> running_{false};
    std::mutex mutex_;
    std::condition_variable cv_;
    
    // === 상태 ===
    uint32_t frame_seq_seen_ = 0;

    // === GStreamer 멤버 변수 ===
    _GstElement* pipeline_ = nullptr;
    _GstAppSrc*  appsrc_ = nullptr;
};

} // namespace flir
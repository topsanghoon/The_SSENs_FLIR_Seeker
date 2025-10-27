#pragma once

#include <thread>
#include <atomic>
#include <string>
#include <mutex>
#include <condition_variable>
#include <memory>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include "components/includes/IR_Frame.hpp"      // IRFrameHandle / FrameGRAY
#include "ipc/mailbox.hpp"                       // SpscMailbox<>
#include "ipc/wake.hpp"                          // WakeHandle (지금은 신호 안 씀)
#include "main_config.hpp"                   // AppConfigPtr
#include "util/common_log.hpp"               // LOGD/LOGI/...

namespace flir {

class IR_TxThread {
public:
    IR_TxThread(std::string name,
                AppConfigPtr cfg,
                SpscMailbox<std::shared_ptr<IRFrameHandle>>& mb,
                WakeHandle& wake);
    ~IR_TxThread();

    IR_TxThread(const IR_TxThread&) = delete;
    IR_TxThread& operator=(const IR_TxThread&) = delete;

    void start();
    void stop();
    void join();

private:
    void run();
    void wait_for_frame();                 // latest_seq() 폴링
    bool init_pipeline();
    void teardown_pipeline();
    void push_frame_to_gst(const std::shared_ptr<IRFrameHandle>& handle);

private:
    // 의존성
    std::string name_;
    AppConfigPtr cfg_;
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& mb_;
    WakeHandle& wake_;                     // 현재는 사용하지 않지만 인터페이스 유지

    // 스레드
    std::thread th_;
    std::atomic<bool> running_{false};

    // 상태
    uint32_t frame_seq_seen_ = 0;

    // GStreamer
    GstElement* pipeline_ = nullptr;
    GstAppSrc*  appsrc_   = nullptr;
};

} // namespace flir

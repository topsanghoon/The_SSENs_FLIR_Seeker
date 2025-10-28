//EO_TxThread.hpp
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include "main_config.hpp"                          // AppConfig/AppConfigPtr
#include "util/common_log.hpp"                      // LOG*
#include "ipc/mailbox.hpp"                              // SpscMailbox
#include "ipc/wake.hpp"                                 // WakeHandle
#include "ipc/wake_condvar.hpp"                         // WakeHandleCondVar
#include "components/includes/EO_Frame.hpp"             // FrameBGR8, EOFrameHandle

namespace flir {

class EO_TxThread {
public:
    EO_TxThread(std::string name,
                AppConfigPtr cfg,
                SpscMailbox<std::shared_ptr<EOFrameHandle>>& mb,
                WakeHandle& /*unused*/);
    ~EO_TxThread();

    EO_TxThread(const EO_TxThread&) = delete;
    EO_TxThread& operator=(const EO_TxThread&) = delete;

    void start();
    void stop();
    void join();

    // 테스트/프로듀서가 사용할 깨우기 핸들 제공 (원래 코드 호환)
    std::unique_ptr<WakeHandle> create_wake_handle();

private:
    bool initialize_gstreamer();
    void wait_for_frame();
    void push_frame_to_gst(const std::shared_ptr<EOFrameHandle>& handle);
    void run();

private:
    // 주입
    std::string name_;
    AppConfigPtr cfg_;
    SpscMailbox<std::shared_ptr<EOFrameHandle>>& mb_;

    // 스레드/동기화
    std::thread       th_;
    std::atomic<bool> running_{false};
    std::condition_variable cv_;
    std::mutex              mutex_;
    uint32_t frame_seq_seen_{0};

    // GStreamer
    GstElement* pipeline_{nullptr};
    GstAppSrc*  appsrc_{nullptr};
};

} // namespace flir

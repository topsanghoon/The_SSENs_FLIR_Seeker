#pragma once
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <functional>

#include "ipc/mailbox.hpp"
#include "ipc/wake.hpp"
#include "main_config.hpp"           // AppConfig/AppConfigPtr
#include "components/includes/EO_Frame.hpp"

namespace flir {

class EO_CaptureThread {
public:
    EO_CaptureThread(std::string name,
                     SpscMailbox<std::shared_ptr<EOFrameHandle>>& out_tx,
                     SpscMailbox<std::shared_ptr<EOFrameHandle>>& out_aru,
                     std::unique_ptr<WakeHandle> wake,
                     AppConfigPtr cfg);
    ~EO_CaptureThread();

    void start();
    void stop();
    void join();

    void set_aru_sink(std::function<void(std::shared_ptr<EOFrameHandle>)> sink) {
        aruco_sink_ = std::move(sink);
    }
    void set_aru_wake(std::unique_ptr<WakeHandle> w) { aruco_wake_ = std::move(w); } // ★ 추가

private:
    void run_();
    void push_frame_(std::shared_ptr<EOFrameHandle> h);

    std::string name_;
    std::thread th_;
    std::atomic<bool> running_{false};
    std::function<void(std::shared_ptr<EOFrameHandle>)> aruco_sink_;

    SpscMailbox<std::shared_ptr<EOFrameHandle>>& out_tx_;
    SpscMailbox<std::shared_ptr<EOFrameHandle>>& out_aru_;
    std::unique_ptr<WakeHandle> wake_;
    AppConfigPtr cfg_;

    uint64_t seq_{0};
    std::unique_ptr<WakeHandle> aruco_wake_;   // ★ 추가
};

} // namespace flir

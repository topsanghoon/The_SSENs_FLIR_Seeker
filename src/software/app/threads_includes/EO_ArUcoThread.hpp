#pragma once
#include <atomic>
#include <thread>
#include <memory>
#include <array>
#include <condition_variable>
#include <opencv2/core.hpp>

#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/event_bus.hpp"
#include "components/includes/EO_Frame.hpp"
#include "main_config.hpp"  // GuidanceConfig

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

    struct Config {
        // period_ms는 이제 "주기 송신"이 아니라, 내부 FSM 등에 쓰일 수 있는 보조 시간 단위로만 남겨둠
        int period_ms     = 20;   // 50Hz 상수, 필요시 내부 계산에만 사용
        int sd_quiesce_ms = 200;
        int sd_park_ms    = 400;

        // ★ 전환 기준(상위 AppConfig.guidance 주입값을 복사 보관)
        GuidanceConfig guidance{};
        // EO 프레임 크기(비율 판단용)
        int eo_w{640};
        int eo_h{480};
    };
    EO_ArUcoThread(SpscMailbox<std::shared_ptr<EOFrameHandle>>& eo_mb,
                   IArucoPreprocessor& preproc,
                   IArucoDetector& detector,
                   IEventBus& bus);

    void start();
    void stop();
    void join();

    void onFrameArrived(std::shared_ptr<EOFrameHandle> h);
    std::unique_ptr<WakeHandle> create_wake_handle(); // ★ 추가

private:
    void run();
    void wait_until_ready();
    void on_frame(const std::shared_ptr<EOFrameHandle>& h);

    void emit_aruco(int id,
                    const std::array<cv::Point2f,4>& corners,
                    const cv::Rect2f& box,
                    uint64_t ts_ns, uint32_t frame_seq);

    bool is_big_enough(int bw, int bh) const;

    SpscMailbox<std::shared_ptr<EOFrameHandle>>& eo_mb_;
    IArucoPreprocessor& preproc_;
    IArucoDetector& detector_;
    IEventBus& bus_;
    Config                        cfg_;
    uint8_t toFindAruco = 1;

    std::thread th_;
    std::atomic<bool> running_{false};
    uint32_t frame_seq_seen_{0};

    std::condition_variable cv_;   // ★ 추가
    std::mutex              m_;    // ★ 추가
};

} // namespace flir

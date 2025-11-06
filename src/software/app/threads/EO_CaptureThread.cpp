#include "threads_includes/EO_CaptureThread.hpp"
#include "components/includes/EO_Frame.hpp"
#include "util/common_log.hpp"
#include "guidance_mode.hpp"

#include <opencv2/opencv.hpp>
#include <chrono>

namespace flir {

namespace { constexpr const char* TAG = "EO_Cap"; }

struct EOOwnedHandle final : public EOFrameHandle {
    std::shared_ptr<std::vector<uint8_t>> owner;
    FrameBGR8 frame;
    EOOwnedHandle(const cv::Mat& bgr) {
        const int w = bgr.cols, h = bgr.rows, step = static_cast<int>(bgr.step);
        owner = std::make_shared<std::vector<uint8_t>>(step * h);
        std::memcpy(owner->data(), bgr.data, owner->size());
        frame.width = w; frame.height = h; frame.step = step; frame.data = owner->data();
        p = &frame;
    }
};

EO_CaptureThread::EO_CaptureThread(std::string name,
                                   SpscMailbox<std::shared_ptr<EOFrameHandle>>& out_tx,
                                   SpscMailbox<std::shared_ptr<EOFrameHandle>>& out_aru,
                                   std::unique_ptr<WakeHandle> wake,
                                   AppConfigPtr cfg)
: name_(std::move(name)), out_tx_(out_tx), out_aru_(out_aru), wake_(std::move(wake)), cfg_(std::move(cfg)) {}

EO_CaptureThread::~EO_CaptureThread() { stop(); join(); }

void EO_CaptureThread::start() { if (running_.exchange(true)) return; th_ = std::thread(&EO_CaptureThread::run_, this); }
void EO_CaptureThread::stop()  { running_.store(false); }
void EO_CaptureThread::join()  { if (th_.joinable()) th_.join(); }

static std::shared_ptr<EOFrameHandle> make_bgr8_handle_(const cv::Mat& bgr) {
    return std::static_pointer_cast<EOFrameHandle>(std::make_shared<EOOwnedHandle>(bgr));
}

void EO_CaptureThread::push_frame_(std::shared_ptr<EOFrameHandle> h) {
    if (!h) return;

    // 1) TX로는 항상
    out_tx_.push(h);
    if (wake_) wake_->signal();

    // 2) 단계 기반 라우팅: Midcourse에선 ArUco로
    auto phase = GuidanceState::phase().load(std::memory_order_relaxed);
    if (phase == GuidancePhase::Midcourse) {
        if (aruco_sink_) {
            aruco_sink_(h);
        } else {
            out_aru_.push(std::move(h));
        }
    }
}

void EO_CaptureThread::run_() {
    using namespace std::chrono_literals;

    const int W = cfg_->eo_tx.frame.width;
    const int H = cfg_->eo_tx.frame.height;
    const int FPS = std::max(1, cfg_->eo_tx.fps);

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        LOGE(TAG, "failed to open camera(0). Falling back to color bars.");
    } else {
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  W);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, H);
        cap.set(cv::CAP_PROP_FPS,          FPS);
    }

    const auto period = std::chrono::milliseconds(1000 / FPS);

    // 1초 주기 처리 로그용
    auto log_t0 = std::chrono::steady_clock::now();
    uint64_t last_seq = 0;

    while (running_.load(std::memory_order_relaxed)) {
        auto t0 = std::chrono::steady_clock::now();

        cv::Mat frame_bgr;
        if (cap.isOpened()) {
            if (!cap.read(frame_bgr) || frame_bgr.empty()) continue;
            if (frame_bgr.cols != W || frame_bgr.rows != H) cv::resize(frame_bgr, frame_bgr, {W,H});
        } else {
            frame_bgr = cv::Mat(H, W, CV_8UC3);
            for (int y=0; y<H; ++y) {
                auto* row = frame_bgr.ptr<cv::Vec3b>(y);
                for (int x=0; x<W; ++x) {
                    int band = (x * 7) / W;
                    static const cv::Vec3b colors[7] = {{255,255,255},{255,255,0},{0,255,255},{0,255,0},{255,0,255},{255,0,0},{0,0,255}};
                    row[x] = colors[band];
                }
            }
        }

        auto h = make_bgr8_handle_(frame_bgr);
        h->seq = ++seq_;
        push_frame_(std::move(h));

        // — 1초 주기 처리 로그 —
        auto now = std::chrono::steady_clock::now();
        if (now - log_t0 >= 1s) {
            auto cur = seq_;
            LOGI(TAG, "EO stats: fps=%llu (total=%llu)", static_cast<unsigned long long>(cur - last_seq),
                 static_cast<unsigned long long>(cur));
            last_seq = cur;
            log_t0 = now;
        }

        auto dt = std::chrono::steady_clock::now() - t0;
        if (dt < period) std::this_thread::sleep_for(period - dt);
    }
    LOGI(TAG, "run() exit");
}

} // namespace flir

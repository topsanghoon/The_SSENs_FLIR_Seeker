#include "threads_includes/EO_CaptureThread.hpp"
#include "components/includes/EO_Frame.hpp"
#include "util/common_log.hpp"
#include "guidance_mode.hpp"

#include <opencv2/opencv.hpp>
#include <chrono>
#include <vector>
#include <cstring>
#include <limits>

namespace flir {

namespace { constexpr const char* TAG = "EO_Cap"; }

// ──────────────────────────────────────────────────────────────
// 안전한 핸들: size_t 기반, 입력검사, 연속/행단위 복사, 예외 방어
// ──────────────────────────────────────────────────────────────
struct EOOwnedHandle final : public EOFrameHandle {
    std::shared_ptr<std::vector<uint8_t>> owner;
    FrameBGR8 frame;
    explicit EOOwnedHandle(int w, int h, size_t step_bytes) {
        const size_t bytes = step_bytes * static_cast<size_t>(h);
        owner = std::make_shared<std::vector<uint8_t>>(bytes);
        frame.width = w; frame.height = h; frame.step = static_cast<int>(step_bytes);
        frame.data = owner->data();
        p = &frame;
    }
};

static std::shared_ptr<EOFrameHandle> make_bgr8_handle_safe_(const cv::Mat& bgr) {
    try {
        if (bgr.empty() || bgr.type() != CV_8UC3) {
            LOGE(TAG, "bad input: empty=%d type=%d", (int)bgr.empty(), bgr.type());
            return nullptr;
        }
        const int    w    = bgr.cols;
        const int    h    = bgr.rows;
        const size_t step = bgr.step; // size_t 유지
        // 기본 유효성 (step >= 3*w)
        if (w <= 0 || h <= 0 || step < static_cast<size_t>(w) * 3) {
            LOGE(TAG, "invalid dims/step: w=%d h=%d step=%zu", w, h, step);
            return nullptr;
        }
        // 초과 방지 상한 (64MB)
        const size_t bytes = step * static_cast<size_t>(h);
        if (bytes > (64ull << 20)) {
            LOGE(TAG, "frame too large: %zu bytes", bytes);
            return nullptr;
        }

        auto hdl = std::make_shared<EOOwnedHandle>(w, h, step);

        if (bgr.isContinuous()) {
            std::memcpy(hdl->owner->data(), bgr.data, bytes);
        } else {
            uint8_t*       dst = hdl->owner->data();
            const uint8_t* src = bgr.data;
            for (int y = 0; y < h; ++y) {
                std::memcpy(dst + static_cast<size_t>(y) * step,
                            src + static_cast<size_t>(y) * step,
                            step);
            }
        }
        return std::static_pointer_cast<EOFrameHandle>(hdl);
    } catch (const std::exception& e) {
        LOGE(TAG, "make_bgr8_handle_safe_ exception: %s", e.what());
        return nullptr;
    }
}

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

void EO_CaptureThread::push_frame_(std::shared_ptr<EOFrameHandle> h) {
    if (!h) return;

    // 1) TX (기존)
    out_tx_.push(h);
    if (wake_) wake_->signal();

    // 2) 단계 기반 라우팅: Midcourse → ArUco
    auto phase = GuidanceState::phase().load(std::memory_order_relaxed);
    if (phase == GuidancePhase::Midcourse) {
        if (aruco_sink_) {
            aruco_sink_(h);               // onFrameArrived() 경유 → 내부에서 notify
        } else {
            out_aru_.push(h);             // 메일박스 경로
            if (aruco_wake_) aruco_wake_->signal();   // ★ 직접 깨움
        }
    }
}

void EO_CaptureThread::run_() {
    using namespace std::chrono_literals;

    const int W   = cfg_->eo_tx.frame.width;
    const int H   = cfg_->eo_tx.frame.height;
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

    // 1초 주기 처리 로그
    auto log_t0  = std::chrono::steady_clock::now();
    uint64_t last_seq = 0;

    while (running_.load(std::memory_order_relaxed)) {
        auto t0 = std::chrono::steady_clock::now();

        cv::Mat frame_bgr;
        if (cap.isOpened()) {
            if (!cap.read(frame_bgr) || frame_bgr.empty()) {
                // 드물게 카메라가 빈 프레임을 줄 수 있음 → 다음 루프
                continue;
            }
            if (frame_bgr.cols != W || frame_bgr.rows != H) {
                cv::resize(frame_bgr, frame_bgr, {W, H});
            }
        } else {
            // 컬러바 fallback
            frame_bgr = cv::Mat(H, W, CV_8UC3);
            for (int y = 0; y < H; ++y) {
                auto* row = frame_bgr.ptr<cv::Vec3b>(y);
                for (int x = 0; x < W; ++x) {
                    int band = (x * 7) / W;
                    static const cv::Vec3b colors[7] = {
                        {255,255,255},{255,255,0},{0,255,255},
                        {0,255,0},{255,0,255},{255,0,0},{0,0,255}
                    };
                    row[x] = colors[band];
                }
            }
        }

        auto h = make_bgr8_handle_safe_(frame_bgr);
        if (!h) {
            // 핸들 생성 실패 → 다음 루프
            continue;
        }
        h->seq = ++seq_;
        push_frame_(std::move(h));

        // — 1초 주기 처리 로그 —
        auto now = std::chrono::steady_clock::now();
        if (now - log_t0 >= 1s) {
            auto cur = seq_;
            LOGI(TAG, "EO stats: fps=%llu (total=%llu)",
                 static_cast<unsigned long long>(cur - last_seq),
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

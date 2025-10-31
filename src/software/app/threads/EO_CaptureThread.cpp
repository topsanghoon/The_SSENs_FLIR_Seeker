// threads_includes/EO_CaptureThread.cpp
#include "threads_includes/EO_CaptureThread.hpp"
#include <cstdio>
#include <thread>
using namespace std::chrono_literals;

namespace flir {

EO_CaptureThread::EO_CaptureThread(std::string name,
                                   SpscMailbox<std::shared_ptr<EOFrameHandle>>& out,
                                   std::unique_ptr<WakeHandle> wake,
                                   AppConfigPtr app)
: name_(std::move(name)), out_(out), wake_(std::move(wake)), app_(std::move(app)) {}

EO_CaptureThread::~EO_CaptureThread() { stop(); join(); }

bool EO_CaptureThread::init_cam() {
    int dev = 0; // 필요 시 AppConfig에 device_id 추가
    if (!cap_.open(dev, cv::CAP_V4L2)) return false;

    // 원하는 설정 주입
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  app_->eo_tx.frame.width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, app_->eo_tx.frame.height);
    cap_.set(cv::CAP_PROP_FPS,          app_->eo_tx.fps);

    // 실제 협상 결과 확인. 불일치면 실패 처리(런타임 resize 금지)
    int got_w = (int)cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    int got_h = (int)cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    int got_f = (int)cap_.get(cv::CAP_PROP_FPS);

    if (got_w != app_->eo_tx.frame.width || got_h != app_->eo_tx.frame.height) {
        std::fprintf(stderr,
            "[EO_Cap][ERR] camera negotiated %dx%d@%dfps (wanted %dx%d@%dfps). Abort.\n",
            got_w, got_h, got_f, app_->eo_tx.frame.width, app_->eo_tx.frame.height, app_->eo_tx.fps);
        return false;
    }

    std::fprintf(stderr, "[EO_Cap] camera ready %dx%d@%dfps\n", got_w, got_h, got_f);
    return true;
}

void EO_CaptureThread::close_cam() {
    if (cap_.isOpened()) cap_.release();
}

void EO_CaptureThread::start() {
    if (running_.exchange(true)) return;
    th_ = std::thread(&EO_CaptureThread::run, this);
}

void EO_CaptureThread::stop() { running_.store(false); }
void EO_CaptureThread::join() { if (th_.joinable()) th_.join(); }

std::shared_ptr<EOFrameHandle> EO_CaptureThread::make_handle(const cv::Mat& bgr) {
    auto h = std::make_shared<EOMatHandle>();
    h->keep = std::make_shared<cv::Mat>(bgr.clone());
    h->owned.data   = h->keep->data;
    h->owned.width  = h->keep->cols;
    h->owned.height = h->keep->rows;
    h->owned.step   = static_cast<int>(h->keep->step);
    return h;
}

void EO_CaptureThread::run() {
    if (!init_cam()) {
        std::fprintf(stderr, "[EO_Cap] open/param fail\n");
        running_.store(false);
        return;
    }

    cv::Mat m;
    uint64_t cnt = 0;

    while (running_) {
        // Producer 게이트: 중기 단계에서만 캡처/전달
        if (!eo_enabled()) { std::this_thread::sleep_for(2ms); continue; }

        if (!cap_.read(m) || m.empty()) continue;

        auto h = make_handle(m);
        out_.push(h);
        if (wake_) wake_->signal();

        // 최소 디버그(90프레임마다 1줄)
        if ((++cnt % 90) == 0) {
            std::fprintf(stderr, "[EO_Cap] frames=%llu\n", (unsigned long long)cnt);
        }
    }

    close_cam();
}

} // namespace flir

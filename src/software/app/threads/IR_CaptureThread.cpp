// threads_includes/IR_CaptureThread.cpp
#include "threads_includes/IR_CaptureThread.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <cstdio>
#include <thread>
using namespace std::chrono_literals;

namespace flir {

static inline bool is_discard(const uint8_t* p) { return (p[0]==0x0F) && ((p[1]&0xF0)==0xF0); }
static inline int  line_num (const uint8_t* p) { return ((p[0]&0x0F)<<8) | p[1]; } // 0..59

IR_CaptureThread::IR_CaptureThread(std::string name,
                                   SpscMailbox<std::shared_ptr<IRFrameHandle>>& out,
                                   AppConfigPtr app)
: name_(std::move(name)), out_(out), app_(std::move(app))
{
    const int W = app_->ir_tx.frame.width;
    const int H = app_->ir_tx.frame.height;
    frame_buf_.resize(W * H);
    pkt_.resize(164);
}

IR_CaptureThread::~IR_CaptureThread() { stop(); join(); }

void IR_CaptureThread::start() {
    if (running_.exchange(true)) return;
    th_ = std::thread(&IR_CaptureThread::run, this);
}

void IR_CaptureThread::stop() { running_.store(false); }

void IR_CaptureThread::join() { if (th_.joinable()) th_.join(); }

bool IR_CaptureThread::init_spi() {
    spi_fd_ = ::open("/dev/spidev0.0", O_RDWR); // 필요시 AppConfig로 경로/속도 확장
    if (spi_fd_ < 0) return false;
    uint8_t mode = SPI_MODE_3, bits = 8; uint32_t speed = 20000000;
    ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    return true;
}

void IR_CaptureThread::close_spi() {
    if (spi_fd_ >= 0) { ::close(spi_fd_); spi_fd_ = -1; }
}

bool IR_CaptureThread::read_packet(uint8_t* buf, size_t len) {
    if (spi_fd_ < 0) return false;
    const ssize_t n = ::read(spi_fd_, buf, len);
    return n == (ssize_t)len;
}

bool IR_CaptureThread::capture_frame_25() {
    // Lepton 2.5: 80x60 → 60 패킷
    const int W = 80, H = 60;
    int lines = 0;
    while (lines < H && running_) {
        if (!read_packet(pkt_.data(), pkt_.size())) return false;
        if (is_discard(pkt_.data())) continue;
        int ln = line_num(pkt_.data()); if (ln < 0 || ln >= 60) continue;
        std::memcpy(&frame_buf_[ln * W], pkt_.data() + 4, W * 2);
        ++lines;
    }
    return (lines == H);
}

bool IR_CaptureThread::capture_frame_30() {
    // Lepton 3.0: 160x120 → 4 세그먼트 × 60 패킷
    const int W = 160, segH = 60;
    static thread_local std::vector<uint16_t> seg(W * segH);
    for (int s = 0; s < 4 && running_; ++s) {
        int lines = 0;
        while (lines < segH && running_) {
            if (!read_packet(pkt_.data(), pkt_.size())) return false;
            if (is_discard(pkt_.data())) continue;
            int ln = line_num(pkt_.data()); if (ln < 0 || ln >= 60) continue;
            std::memcpy(&seg[ln * W], pkt_.data() + 4, W * 2);
            ++lines;
        }
        std::memcpy(&frame_buf_[s * segH * W], seg.data(), W * segH * 2);
    }
    return true;
}

bool IR_CaptureThread::capture_frame_any() {
    const auto& sz = app_->ir_tx.frame;
    if (sz.width == 80 && sz.height == 60) return capture_frame_25();
    return capture_frame_30();
}

std::shared_ptr<IRFrameHandle> IR_CaptureThread::make_handle() {
    const int W = app_->ir_tx.frame.width;
    const int H = app_->ir_tx.frame.height;

    // frame_buf_ → Mat 래핑 후 clone해서 수명 보장
    cv::Mat tmp(H, W, CV_16UC1, frame_buf_.data());
    auto h = std::make_shared<IRMatHandle>();
    h->keep = std::make_shared<cv::Mat>(tmp.clone());

    h->owned.data   = reinterpret_cast<uint16_t*>(h->keep->data);
    h->owned.width  = h->keep->cols;
    h->owned.height = h->keep->rows;
    h->owned.step   = static_cast<int>(h->keep->step);
    return h;
}

void IR_CaptureThread::run() {
    if (!init_spi()) {
        std::fprintf(stderr, "[IR_Cap] SPI open fail\n");
        running_.store(false);
        return;
    }

    uint64_t cnt_log = 0;

    while (running_) {
        // Producer 게이트: 종말 단계에서만 캡처/전달
        if (!ir_enabled()) { std::this_thread::sleep_for(2ms); continue; }

        if (!capture_frame_any()) { error_count_++; continue; }

        auto h = make_handle();
        out_.push(h);
        frame_count_++;

        // 최소 디버그(60프레임마다 1줄)
        if ((++cnt_log % 60) == 0) {
            std::fprintf(stderr, "[IR_Cap] frames=%llu (%dx%d)\n",
                (unsigned long long)frame_count_.load(),
                app_->ir_tx.frame.width, app_->ir_tx.frame.height);
        }
    }

    close_spi();
}

} // namespace flir

#include "threads_includes/IR_CaptureThread.hpp"

#include <iostream>
#include <cstring>
#include <cerrno>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/i2c-dev.h>   // I2C_SLAVE

namespace flir {

IR_CaptureThread::IR_CaptureThread(
    std::string name,
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& out_tx,
    SpscMailbox<std::shared_ptr<IRFrameHandle>>& out_trk,
    AppConfigPtr cfg)
: name_(std::move(name))
, out_tx_(out_tx)
, out_trk_(out_trk)
, cfg_(std::move(cfg))
, segment_buffer_(vospi::PACKETS_PER_SEGMENT * vospi::PAYLOAD_SIZE)
, frame_buffer_(vospi::FRAME_WIDTH * vospi::FRAME_HEIGHT)
{}

IR_CaptureThread::~IR_CaptureThread() {
    stop();
    join();
    cleanup_spi();
}

void IR_CaptureThread::start() {
    if (running_.exchange(true)) return;

    if (!initialize_spi()) {
        running_.store(false);
        throw std::runtime_error("IR_CaptureThread: SPI init failed");
    }
    th_ = std::thread(&IR_CaptureThread::run, this);
}

void IR_CaptureThread::stop() {
    running_.store(false);
}

void IR_CaptureThread::join() {
    if (th_.joinable()) th_.join();
}

bool IR_CaptureThread::initialize_spi() {
    // 기본값: /dev/spidev1.0, 12.5MHz (cfg의 ir_tx.fps만 사용; 속도/경로는 여기서 설정)
    const std::string dev = "/dev/spidev1.0";
    const uint32_t    hz  = 12500000;

    spi_fd_ = ::open(dev.c_str(), O_RDWR);
    if (spi_fd_ < 0) {
        std::cerr << "[" << name_ << "] open " << dev
                  << " failed: " << std::strerror(errno) << "\n";
        return false;
    }

    uint8_t mode = SPI_MODE_3; // CPOL=1, CPHA=1
    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0) {
        std::cerr << "[" << name_ << "] set mode failed\n";
        cleanup_spi(); return false;
    }

    uint8_t bits = 8;
    if (ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        std::cerr << "[" << name_ << "] set bits-per-word failed\n";
        cleanup_spi(); return false;
    }

    uint32_t set_hz = hz;
    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &set_hz) < 0) {
        std::cerr << "[" << name_ << "] set speed failed\n";
        cleanup_spi(); return false;
    }

    uint8_t rd_mode=0; uint32_t rd_hz=0;
    ioctl(spi_fd_, SPI_IOC_RD_MODE, &rd_mode);
    ioctl(spi_fd_, SPI_IOC_RD_MAX_SPEED_HZ, &rd_hz);
    std::cout << "[" << name_ << "] SPI ok: mode=0x" << std::hex << int(rd_mode)
              << std::dec << " speed=" << (rd_hz?rd_hz:set_hz) << "Hz\n";
    return true;
}

void IR_CaptureThread::cleanup_spi() {
    if (spi_fd_ >= 0) {
        ::close(spi_fd_);
        spi_fd_ = -1;
    }
}

void IR_CaptureThread::reset_camera() {
    std::cout << "[" << name_ << "] Lepton reboot via I2C...\n";

    if (spi_fd_ >= 0) { ::close(spi_fd_); spi_fd_ = -1; }

    // Lepton CCI: /dev/i2c-0 @ 0x2a
    int i2c = ::open("/dev/i2c-0", O_RDWR);
    if (i2c >= 0) {
        if (ioctl(i2c, I2C_SLAVE, 0x2a) >= 0) {
            // 간단한 OEM reboot 예시 (LEPTON_OEM 참고)
            uint8_t reboot_cmd[4] = {0x08, 0x02, 0x00, 0x00};
            (void)::write(i2c, reboot_cmd, 4);
            usleep(750000);
        } else {
            std::cerr << "[" << name_ << "] I2C_SLAVE set failed\n";
        }
        ::close(i2c);
    } else {
        std::cerr << "[" << name_ << "] open /dev/i2c-0 failed\n";
    }

    usleep(200000);
    (void)initialize_spi();
}

void IR_CaptureThread::run() {
    std::cout << "[" << name_ << "] run() start\n";

    using clock = std::chrono::steady_clock;
    const int fps = std::max(1, cfg_ ? cfg_->ir_tx.fps : 9);
    const auto period = std::chrono::microseconds(1000000 / fps);

    auto next_t = clock::now();

    while (running_.load(std::memory_order_relaxed)) {
        bool ok = false;
        try {
            ok = capture_vospi_frame();
        } catch (...) {
            ok = false;
        }

        if (ok) {
            auto h = create_frame_handle();
            if (h) {
                // fan-out
                
                out_tx_.push(h);
                out_trk_.push(std::move(h));
                frame_count_.fetch_add(1);
                std::cout << "[IR_Cap] \n";
            }
        } else {
            error_count_.fetch_add(1);
        }

        next_t += period;
        std::this_thread::sleep_until(next_t);
    }

    std::cout << "[" << name_ << "] run() exit\n";
}

bool IR_CaptureThread::capture_vospi_frame() {
    // Lepton 2.5 → segment 1개
    if (!capture_segment(0)) return false;
    reconstruct_frame();
    return true;
}

bool IR_CaptureThread::capture_segment(int /*seg_id*/) {
    uint8_t pkt[vospi::PACKET_SIZE];
    int resets = 0;

    for (int expected = 0; expected < vospi::PACKETS_PER_SEGMENT; ) {
        if (!read_vospi_packet(pkt)) return false;

        if (is_discard_packet(pkt)) {
            discard_count_.fetch_add(1);
            expected = 0;
            if (++resets % 60 == 0) usleep(1000);
            continue;
        }

        const int line = get_packet_line_number(pkt);
        if (line != expected || line < 0 || line >= vospi::LINES_PER_SEGMENT) {
            expected = 0;
            if (++resets % 60 == 0) usleep(1000);
            continue;
        }

        // payload copy (160B)
        std::memcpy(&segment_buffer_[line * vospi::PAYLOAD_SIZE],
                    &pkt[4], vospi::PAYLOAD_SIZE);
        ++expected;
    }
    return true;
}

bool IR_CaptureThread::read_vospi_packet(uint8_t* buf) {
    struct spi_ioc_transfer tr{};
    tr.tx_buf        = 0; // no TX
    tr.rx_buf        = reinterpret_cast<uintptr_t>(buf);
    tr.len           = vospi::PACKET_SIZE;
    tr.speed_hz      = 20000000;
    tr.bits_per_word = 8;
    tr.cs_change     = 1; // VoSPI 동기화에 중요
    tr.delay_usecs   = 50;

    int r = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    return (r >= 0);
}

void IR_CaptureThread::reconstruct_frame() {
    // segment_buffer_ → frame_buffer_ (BE→LE, Raw14 마스킹)
    for (int y = 0; y < vospi::FRAME_HEIGHT; ++y) {
        const uint8_t* src = &segment_buffer_[y * vospi::PAYLOAD_SIZE];
        uint16_t* dst      = &frame_buffer_[y * vospi::FRAME_WIDTH];
        for (int x = 0; x < vospi::PIXELS_PER_LINE; ++x) {
            const uint16_t be = (uint16_t(src[2*x]) << 8) | uint16_t(src[2*x + 1]);
            dst[x] = be & 0x3FFF; // Raw14 데이터
        }
    }
}

std::shared_ptr<IRFrameHandle> IR_CaptureThread::create_frame_handle() {
    // Mat view → clone 보유
    cv::Mat raw16(vospi::FRAME_HEIGHT, vospi::FRAME_WIDTH, CV_16UC1, frame_buffer_.data());
    auto h = std::make_shared<IRMatHandle>();
    h->keep = std::make_shared<cv::Mat>(raw16.clone());

    auto& owned = h->owned;
    owned.data   = reinterpret_cast<uint16_t*>(h->keep->data);
    owned.width  = h->keep->cols;
    owned.height = h->keep->rows;
    owned.step   = static_cast<int>(h->keep->step);

    h->seq = seq_.fetch_add(1);
    h->ts  = now_ns();
    return h;
}

bool IR_CaptureThread::is_discard_packet(const uint8_t* p) {
    return (p[0] & 0x0F) == 0x0F;
}

int IR_CaptureThread::get_packet_line_number(const uint8_t* p) {
    return int(p[1]); // 0..59
}

uint64_t IR_CaptureThread::now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(high_resolution_clock::now().time_since_epoch()).count();
}

} // namespace flir

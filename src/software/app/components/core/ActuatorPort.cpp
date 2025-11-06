// components/core/ActuatorPort.cpp
#include "components/includes/ActuatorPort.hpp"

// POSIX / UART
#include <fcntl.h>      // open
#include <unistd.h>     // write, close
#include <termios.h>    // termios, speed_t, cfsetispeed/cfsetospeed
#include <stdexcept>    // std::runtime_error
#include <cstring>      // strerror
#include <errno.h>      // errno
#include <cstdint>      // int8_t
#include <algorithm>    // std::clamp
#include <string>
#include <iostream>

// CSV 로깅
#include "util/csv_sink.hpp"   // CSV_LOG_SIMPLE(tag, event, seq, v1,v2,v3,v4, note)

namespace flir {

namespace {
constexpr const char* TAG = "UART";
// helper: int -> int8_t 범위로 클램프
inline int8_t clamp_to_i8(int v) {
    if (v < -128) return static_cast<int8_t>(-128);
    if (v >  127) return static_cast<int8_t>( 127);
    return static_cast<int8_t>(v);
}
}

// --- 생성자 ---
UART_ActuatorPort::UART_ActuatorPort(const std::string& device_path, speed_t baud_rate)
{
    // O_WRONLY: 쓰기 전용, O_NOCTTY: 제어 터미널 아님, O_NDELAY: non-blocking
    fd_ = ::open(device_path.c_str(), O_WRONLY | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        CSV_LOG_SIMPLE(TAG, "OPEN_FAIL", 0, (double)errno, 0,0,0,
                       std::string("dev=" + device_path + " err=" + std::strerror(errno)).c_str());
        throw std::runtime_error("Failed to open UART device: " + device_path + " - " + std::string(std::strerror(errno)));
    }

    struct termios options{};
    if (::tcgetattr(fd_, &options) != 0) {
        int e = errno;
        ::close(fd_); fd_ = -1;
        CSV_LOG_SIMPLE(TAG, "TCGETATTR_FAIL", 0, (double)e, 0,0,0, "tcgetattr");
        throw std::runtime_error("tcgetattr failed: " + std::string(std::strerror(e)));
    }

    // Baudrate
    ::cfsetispeed(&options, baud_rate);
    ::cfsetospeed(&options, baud_rate);

    // 8N1
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;

    // Raw 모드
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // SW flow off

    if (::tcsetattr(fd_, TCSANOW, &options) != 0) {
        int e = errno;
        ::close(fd_); fd_ = -1;
        CSV_LOG_SIMPLE(TAG, "TCSETATTR_FAIL", 0, (double)e, 0,0,0, "tcsetattr");
        throw std::runtime_error("Failed to configure UART: " + std::string(std::strerror(e)));
    }

    CSV_LOG_SIMPLE(TAG, "OPEN_OK", 0, (double)baud_rate, 0,0,0, device_path.c_str());
}

// --- 소멸자 ---
UART_ActuatorPort::~UART_ActuatorPort() {
    if (fd_ != -1) {
        ::close(fd_);
        CSV_LOG_SIMPLE(TAG, "CLOSE", 0, 0,0,0,0, "");
        fd_ = -1;
    }
}

// --- 비차단 1바이트 전송 ---
void UART_ActuatorPort::write_nonblock(const CtrlCmd& cmd) {
    if (quiesce_.load() || fd_ < 0) {
        CSV_LOG_SIMPLE(TAG, "TX_SKIP", 0, (double)quiesce_.load(), 0,0,0, "quiesced or fd<0");
        return;
    }

    // p1을 모터 목표치로 사용 (범위 클램프)
    // CtrlCmd.p1 타입이 float/double일 수 있어 정수 변환
    int desired = static_cast<int>(cmd.p1);
    int8_t motor_val = clamp_to_i8(desired);
    bool clamped = (desired != static_cast<int>(motor_val));

    ssize_t n = ::write(fd_, &motor_val, sizeof(motor_val));
    if (n < 0) {
        int e = errno;
        // 재시도 고려: EAGAIN/EWOULDBLOCK 은 드롭(비차단)
        CSV_LOG_SIMPLE(TAG, "TX_ERR", 0, (double)e, (double)motor_val, 0,0, std::strerror(e));
        return;
    }

    // 성공 CSV (성공 바이트 수와 값 기록, 클램프 여부 note)
    CSV_LOG_SIMPLE(TAG, "TX_OK", 0, (double)n, (double)motor_val, 0,0, clamped ? "clamped" : "");
}

// --- 정지/무시 플래그 ---
void UART_ActuatorPort::set_quiesce(bool q) {
    quiesce_.store(q);
    CSV_LOG_SIMPLE(TAG, "QUIESCE", 0, q ? 1.0 : 0.0, 0,0,0, "");
}

// --- stop_io: 안전정지 바이트 전송(에러 무시) ---
void UART_ActuatorPort::stop_io() {
    std::cout << "before if\n";
    if (fd_ == -1) {
        CSV_LOG_SIMPLE(TAG, "STOP_IO_SKIP", 0, 0,0,0,0, "fd<0");
        return;
    }
    std::cout << "after if\n";
    int desired = static_cast<int>(120);
    int8_t stop_val = clamp_to_i8(desired);
    bool clamped = (desired != static_cast<int>(stop_val));

    ssize_t n = ::write(fd_, &stop_val, sizeof(stop_val));
    CSV_LOG_SIMPLE(TAG, "STOP_IO", 0, (double)stop_val, 0,0,0, "");
}

} // namespace flir

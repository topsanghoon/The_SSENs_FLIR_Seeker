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

// 필요하면 여기서 common_log.hpp 추가해서 LOGE/LOGI 쓰는 것도 가능
// #include "util/common_log.hpp"

namespace flir {

namespace {

// helper: int -> int8_t 범위로 클램프
inline int8_t clamp_to_i8(int v) {
    if (v < -128) return static_cast<int8_t>(-128);
    if (v >  127) return static_cast<int8_t>( 127);
    return static_cast<int8_t>(v);
}

} // anonymous namespace

// --- 생성자 ---
UART_ActuatorPort::UART_ActuatorPort(const std::string& device_path, speed_t baud_rate)
{
    // O_WRONLY: 쓰기 전용, O_NOCTTY: 제어 터미널 아님, O_NDELAY: non-blocking
    fd_ = ::open(device_path.c_str(), O_WRONLY | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        throw std::runtime_error(
            "Failed to open UART device: " + device_path +
            " - " + std::string(std::strerror(errno))
        );
    }

    struct termios options{};
    if (::tcgetattr(fd_, &options) != 0) {
        int e = errno;
        ::close(fd_);
        fd_ = -1;
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
        ::close(fd_);
        fd_ = -1;
        throw std::runtime_error("Failed to configure UART: " + std::string(std::strerror(e)));
    }
}

// --- 소멸자 ---
UART_ActuatorPort::~UART_ActuatorPort() {
    if (fd_ != -1) {
        ::close(fd_);
        fd_ = -1;
    }
}

// --- 비차단 1바이트 전송 ---
void UART_ActuatorPort::write_nonblock(const CtrlCmd& cmd) {
    if (quiesce_.load() || fd_ < 0) {
        return;
    }

    // p1을 모터 목표치로 사용 (범위 클램프)
    int desired = static_cast<int>(cmd.p1);
    int8_t motor_val = clamp_to_i8(desired);

    ssize_t n = ::write(fd_, &motor_val, sizeof(motor_val));
    if (n < 0) {
        // EAGAIN/EWOULDBLOCK 등은 비차단 특성상 그냥 드롭
        return;
    }
}

// --- 정지/무시 플래그 ---
void UART_ActuatorPort::set_quiesce(bool q) {
    quiesce_.store(q);
}

// --- stop_io: 안전정지 바이트 전송(에러 무시) ---
void UART_ActuatorPort::stop_io() {
    if (fd_ == -1) {
        return;
    }
    int desired = 119;          // 설계상 STOP 코드
    int8_t stop_val = clamp_to_i8(desired);
    (void)::write(fd_, &stop_val, sizeof(stop_val));
}

void UART_ActuatorPort::send_start_signal() {
    if (fd_ == -1) {
        return;
    }
    int desired = 109;          // 설계상 START 코드
    int8_t val = clamp_to_i8(desired);
    (void)::write(fd_, &val, sizeof(val));
}

} // namespace flir

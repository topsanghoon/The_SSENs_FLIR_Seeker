// core
#include "components/includes/ActuatorPort.hpp"

// C/C++ system headers for UART
#include <fcntl.h>   // open
#include <unistd.h>  // write, close
#include <stdexcept> // std::runtime_error
#include <cstring>   // strerror
#include <errno.h>   // errno
#include <iostream>  // std::cout, std::cerr
#include <cstdint>   // int8_t

namespace flir {

// --- 생성자 구현 ---
UART_ActuatorPort::UART_ActuatorPort(const std::string& device_path, speed_t baud_rate) {
    // O_WRONLY: 쓰기 전용, O_NOCTTY: 제어 터미널 X, O_NDELAY: Non-blocking
    fd_ = open(device_path.c_str(), O_WRONLY | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        throw std::runtime_error("Failed to open UART device: " + device_path + " - " + strerror(errno));
    }

    struct termios options;
    tcgetattr(fd_, &options); // 현재 설정 가져오기

    // Baudrate 설정 (예: B9600)
    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);

    // 8N1 (8 data bits, No parity, 1 stop bit) 설정
    options.c_cflag |= (CLOCAL | CREAD); // 수신 활성화, 모뎀 제어 무시
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;       // 8 data bits
    options.c_cflag &= ~PARENB;   // No parity
    options.c_cflag &= ~CSTOPB;   // 1 stop bit

    // Raw 모드 (데이터 가공 없이 그대로 전송)
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;

    // 설정 즉시 적용
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
         close(fd_); // Failsafe
         fd_ = -1;
         throw std::runtime_error("Failed to configure UART: " + std::string(strerror(errno)));
    }
    
    std::cout << "[UART] Port " << device_path << " opened at " << baud_rate << " baud.\n";
}

// --- 소멸자 구현 ---
UART_ActuatorPort::~UART_ActuatorPort() {
    if (fd_ != -1) {
        std::cout << "[UART] Port closed.\n";
        close(fd_);
    }
}

// --- write_nonblock 구현 (1바이트 전송) ---
void UART_ActuatorPort::write_nonblock(const CtrlCmd& cmd) {
    if (quiesce_.load() || fd_ < 0) return;

    // 1바이트 전송 로직 (Arduino의 Serial1.read()와 호환)
    int8_t motor_val = static_cast<int8_t>(cmd.p1 * cmd.mode);
    
    ssize_t bytes_written = write(fd_, &motor_val, sizeof(motor_val));

    if (bytes_written < 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
             std::cerr << "[UART] Write Error: " << strerror(errno) << std::endl;
        }
    }
    
    // (디버깅용) 콘솔에도 동일하게 출력
    std::cout << "[UART->ttyUL0] Sent: " << (int)motor_val << '\n';
}

// --- set_quiesce 구현 ---
void UART_ActuatorPort::set_quiesce(bool q) {
    quiesce_.store(q);
    std::cout << "[UART] set_quiesce=" << (q?"true":"false") << "\n";
}

// --- stop_io 구현 ---
void UART_ActuatorPort::stop_io() {
    std::cout << "[UART] stop_io\n";
    if (fd_ != -1) {
        // 멈출 때 '119' (SD) 명령을 보냄
        int8_t stop_val = static_cast<int8_t>(119);
        (void)!::write(fd_, &stop_val, sizeof(stop_val)); // Error ignored on stop
    }
}

} // namespace flir
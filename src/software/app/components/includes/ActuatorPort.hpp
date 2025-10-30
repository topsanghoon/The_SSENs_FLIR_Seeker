#pragma once

#include <atomic>
#include <string>      // std::string
#include <termios.h>   // speed_t
#include "components/includes/ControlDTO.hpp"

namespace flir {

class IActuatorPort {
public:
    virtual ~IActuatorPort() = default;
    virtual void write_nonblock(const CtrlCmd& cmd) = 0;
    virtual void set_quiesce(bool q) = 0;
    virtual void stop_io() = 0;
};


// UART_ActuatorPort가 스스로 포트를 열고 관리하도록 생성자를 변경합니다.
class UART_ActuatorPort : public IActuatorPort {
public:
    // 생성자: 장치 경로와 전송 속도를 받아 스스로 포트를 엽니다.
    UART_ActuatorPort(const std::string& device_path, speed_t baud_rate);
    
    // 소멸자: 포트를 닫습니다. (virtual for base class)
    virtual ~UART_ActuatorPort();

    // 인터페이스 오버라이드
    void write_nonblock(const CtrlCmd& cmd) override;
    void set_quiesce(bool q) override;
    void stop_io() override;

private:
    int fd_{-1}; // File descriptor
    std::atomic<bool> quiesce_{false};
};

} // namespace flir
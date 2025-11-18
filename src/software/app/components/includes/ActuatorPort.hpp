#pragma once
#include <atomic>
#include <string>
#include <termios.h>
#include "components/includes/ControlDTO.hpp"

namespace flir {

class IActuatorPort {
public:
    virtual ~IActuatorPort() = default;
    virtual void write_nonblock(const CtrlCmd& cmd) = 0;
    virtual void set_quiesce(bool q) = 0;
    virtual void stop_io() = 0;
    virtual void send_start_signal() = 0;
};

class UART_ActuatorPort : public IActuatorPort {
public:
    UART_ActuatorPort(const std::string& serial_dev, speed_t baud = B115200);
    ~UART_ActuatorPort() override;

    void write_nonblock(const CtrlCmd& cmd) override;
    void set_quiesce(bool q) override;
    void stop_io() override;
    void send_start_signal() override;

private:
    int fd_{-1};
    std::atomic<bool> quiesce_{false};
};

} // namespace flir

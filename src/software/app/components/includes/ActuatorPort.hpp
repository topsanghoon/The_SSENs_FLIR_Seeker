#pragma once
#include <atomic>
#include "components/includes/ControlDTO.hpp"

namespace flir {

// 액추에이터 포트 인터페이스(예: UART, CAN, SPI 등)
class IActuatorPort {
public:
    virtual ~IActuatorPort() = default;
    virtual void write_nonblock(const CtrlCmd& cmd) = 0;
    virtual void set_quiesce(bool q) = 0;
    virtual void stop_io() = 0;
};

// UART 전송 예시 구현(테스트용)
class UART_ActuatorPort : public IActuatorPort {
public:
    explicit UART_ActuatorPort(int fd = -1) : fd_(fd) {}
    ~UART_ActuatorPort() override = default;

    void write_nonblock(const CtrlCmd& cmd) override;
    void set_quiesce(bool q) override { quiesce_.store(q); }
    void stop_io() override;  // 필요시 fd 정리

    // fd 주입/교체 헬퍼(옵션)
    void set_fd(int fd) { fd_ = fd; }
    int  fd() const { return fd_; }

private:
    int               fd_{-1};
    std::atomic<bool> quiesce_{false};
};

} // namespace flir

// core
#include "components/includes/ActuatorPort.hpp"
#include <unistd.h>
#include <cstring>

namespace flir {

void UART_ActuatorPort::write_nonblock(const CtrlCmd& cmd) {
    if (quiesce_.load()) return;
    if (fd_ < 0) return;

    // 아주 단순한 바이너리 포맷(임시)
    struct Packet { int mode; float p1,p2,p3; } p{cmd.mode, cmd.p1, cmd.p2, cmd.p3};
    // 비차단 가정: fd를 O_NONBLOCK으로 열었다는 전제
    (void)!::write(fd_, &p, sizeof(p)); // 에러 처리는 생략
}

void UART_ActuatorPort::stop_io() {
    // 테스트에선 아무 것도 하지 않음.
    // 실제 적용 시: tcdrain(), close(fd_), 재연결 플래그 등 처리
}

} // namespace flir

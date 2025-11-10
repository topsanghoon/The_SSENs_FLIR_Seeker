#pragma once
#include <cstdint>

namespace flir {

struct CtrlCmd {
    int mode = 0;
    float p1 = 0, p2 = 0, p3 = 0;

    // 기본 동작 명령 (중립)
    static CtrlCmd safe_pose() {
        CtrlCmd c; c.mode = 0;
        return c;
    }

    // 자폭 알림 명령
    static CtrlCmd make_self_destruct() {
        CtrlCmd c;
        c.mode = 119;  
        c.p1 = c.p2 = c.p3 = 119;  // 아두이노 측에서 119를 STOP으로 처리
        return c;
    }

    static CtrlCmd make_phase_switch_terminal() { return CtrlCmd{ /*예:*/ 9001u }; } // ★ 예약 코드
       static CtrlCmd make_start_signal() {
        return CtrlCmd{8001u};
    }

    int to_int() const { return mode; }
};

} // namespace flir
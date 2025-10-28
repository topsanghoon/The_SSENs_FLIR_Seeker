//Meta_TxThread.hpp
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <netinet/in.h>

#include "main_config.hpp"
#include "util/common_log.hpp"
#include "ipc/event_bus.hpp"     // IEventBus, Topic, Event
#include "ipc/mailbox.hpp"       // SpscMailbox<Event>
#include "ipc/ipc_types.hpp"     // EventType 등 (payload는 cpp에서만 구체 해석)

namespace flir {

class Meta_TxThread {
public:
    Meta_TxThread(IEventBus& bus, AppConfigPtr cfg);
    ~Meta_TxThread();

    Meta_TxThread(const Meta_TxThread&) = delete;
    Meta_TxThread& operator=(const Meta_TxThread&) = delete;

    void start();
    void stop();
    void join();

private:
    // IO
    bool init_io_();          // socket + epoll + eventfd + timerfd
    void close_io_();
    void set_meta_target_(const struct sockaddr* sa, socklen_t slen);

    // loop
    void run_();
    void on_eventfd_ready_();
    void on_timerfd_ready_();

    // send helpers (헤더에 외부 패킷 타입 노출 안 함)
    void send_track_(uint64_t ts, uint32_t seq, float x, float y, float w, float h, float score);
    void send_aruco_(uint64_t ts, int id, float x, float y, float w, float h);
    void send_ctrl_(uint64_t ts, uint32_t state_or_cmd);
    void send_hb_(uint64_t ts);
    void send_aruco_full_(uint64_t ts, int id, const cv::Rect2f& box,
                      const std::array<cv::Point2f,4>& c);
    void send_aruco_bbox_(uint64_t ts, int id, const cv::Rect2f& box); // optional

private:
    // 의존성
    IEventBus&   bus_;
    AppConfigPtr cfg_;

    // inbox & wake(eventfd 기반 핸들; 정의는 cpp 내부)
    SpscMailbox<Event> inbox_{128};
    class EfdWakeHandle;
    std::unique_ptr<EfdWakeHandle> wake_;

    // 스레드/상태
    std::thread       th_;
    std::atomic<bool> running_{false};

    // fd들
    int epfd_{-1};
    int efd_{-1};
    int tfd_{-1};
    int sock_{-1};
    bool own_epfd_{false};
    bool own_efd_{false};
    bool own_tfd_{false};
    bool own_sock_{false};

    // 목적지
    sockaddr_storage sa_meta_{};
    socklen_t        sl_meta_{0};

    // 최근 상태(헤더에 도메인 타입 노출 없이 필드만 저장)
    struct { float x{0}, y{0}, w{0}, h{0}, score{0}; uint64_t ts{0}; uint32_t seq{0}; } last_trk_{};
    struct { int id{0}; float x{0}, y{0}, w{0}, h{0}; uint64_t ts{0}; } last_aru_{};
    struct { uint32_t state{0}; uint32_t last_cmd{0}; uint64_t ts{0}; } last_ctl_{};

    uint64_t last_sent_ns_{0};
    int      hb_period_ms_{1000};
};

} // namespace flir

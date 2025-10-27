#include "threads_includes/Meta_TxThread.hpp"
#include "components/includes/MetaWire.hpp"     // 패킷 빌더
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <cerrno>
#include <arpa/inet.h>   // inet_pton (리눅스)
#include <netinet/in.h>  // sockaddr_in
#include <iostream>

#include "ipc/event_bus.hpp"
#include "ipc/ipc_types.hpp"

namespace flir {

// ---- WakeHandle: eventfd 기반 ----
class EfdWakeHandle : public WakeHandle {
public:
    explicit EfdWakeHandle(int efd) : efd_(efd) {}
    void signal() override {
        uint64_t one = 1;
        (void)!::write(efd_, &one, sizeof(one)); // EAGAIN 무시
    }
private:
    int efd_;
};

// ---- 유틸 ----
static inline uint64_t now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}
static inline uint64_t now_ms_epoch() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
static inline void drain_eventfd(int efd) {
    uint64_t cnt;
    while (::read(efd, &cnt, sizeof(cnt)) > 0) { /* drain all */ }
}
static inline void drain_timerfd(int tfd) {
    uint64_t expirations;
    (void)::read(tfd, &expirations, sizeof(expirations));
}
static inline itimerspec make_period_ms(int ms) {
    itimerspec its{};
    its.it_value.tv_sec  = ms / 1000;
    its.it_value.tv_nsec = (ms % 1000) * 1000000LL;
    its.it_interval      = its.it_value; // periodic
    return its;
}

static bool make_sockaddr_ipv4(const char* ip, uint16_t port,
                               sockaddr_storage& ss, socklen_t& sl) {
    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(port);
    if (::inet_pton(AF_INET, ip, &sa.sin_addr) != 1) return false;
    std::memset(&ss, 0, sizeof(ss));
    std::memcpy(&ss, &sa, sizeof(sa));
    sl = sizeof(sa);
    return true;
}

// ---- ctor/lifecycle ----
Meta_TxThread::Meta_TxThread(IEventBus& bus, MetaTxConfig cfg, MetaFds fds)
: bus_(bus),
  cfg_(cfg),
  fds_(fds)
{}

void Meta_TxThread::start() {
    running_.store(true);

    // epoll/efd/tfd 생성
    if (fds_.epfd < 0) { fds_.epfd = ::epoll_create1(EPOLL_CLOEXEC); own_epfd_ = true; }
    if (fds_.efd  < 0) { fds_.efd  = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC); own_efd_ = true; }
    if (fds_.tfd  < 0) { fds_.tfd  = ::timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC); own_tfd_ = true; }

    itimerspec its = make_period_ms(cfg_.hb_period_ms);
    ::timerfd_settime(fds_.tfd, 0, &its, nullptr);

    epoll_event ev{};
    ev.events = EPOLLIN;
    ev.data.fd = fds_.efd;  ::epoll_ctl(fds_.epfd, EPOLL_CTL_ADD, fds_.efd, &ev);
    ev.data.fd = fds_.tfd;  ::epoll_ctl(fds_.epfd, EPOLL_CTL_ADD, fds_.tfd, &ev);

    // 소켓 생성/옵션/바인드(선택)
    if (fds_.sock_meta < 0) {
        int s = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (s >= 0) {
            if (cfg_.sndbuf_bytes > 0) {
                int sz = cfg_.sndbuf_bytes;
                (void)::setsockopt(s, SOL_SOCKET, SO_SNDBUF, &sz, sizeof(sz));
            }
            if (cfg_.local_port != 0) {
                sockaddr_in a{};
                a.sin_family      = AF_INET;
                a.sin_addr.s_addr = INADDR_ANY;
                a.sin_port        = htons(cfg_.local_port);
                if (::bind(s, (sockaddr*)&a, sizeof(a)) != 0) {
                    log_debug("bind failed on :" + std::to_string(cfg_.local_port) +
                              " err=" + strerror(errno));
                } else {
                    log_debug("bound on :" + std::to_string(cfg_.local_port));
                }
            }
            fds_.sock_meta = s;
            own_sock_ = true;
        } else {
            log_debug("socket() failed: " + std::string(strerror(errno)));
        }
    }

    // 목적지 설정
    if (cfg_.remote_port != 0 && cfg_.remote_ip[0] != '\0') {
        sockaddr_storage dst{}; socklen_t dlen = 0;
        if (make_sockaddr_ipv4(cfg_.remote_ip, cfg_.remote_port, dst, dlen)) {
            set_meta_target(reinterpret_cast<sockaddr*>(&dst), dlen);
            log_debug("target set to " + std::string(cfg_.remote_ip) + 
                      ":" + std::to_string(cfg_.remote_port));
        } else {
            log_debug("invalid remote_ip: " + std::string(cfg_.remote_ip));
        }
    } else {
        log_debug("remote target not set");
    }

    // EVT_BUS 구독
    wake_ = std::make_unique<EfdWakeHandle>(fds_.efd);
    bus_.subscribe(Topic::Tracking, &inbox_, wake_.get());
    bus_.subscribe(Topic::Aruco,    &inbox_, wake_.get());
    bus_.subscribe(Topic::Control,  &inbox_, wake_.get());

    log_debug("hb_period_ms=" + std::to_string(cfg_.hb_period_ms) +
              " remote=" + cfg_.remote_ip + ":" + std::to_string(cfg_.remote_port));

    th_ = std::thread(&Meta_TxThread::run, this);
}

void Meta_TxThread::stop() {
    running_.store(false);
    if (fds_.efd >= 0) {
        EfdWakeHandle tmp(fds_.efd);
        tmp.signal(); // 깨워서 종료 루프로
    }
}

void Meta_TxThread::join() {
    if (th_.joinable()) th_.join();
    bus_.unsubscribe(&inbox_);
    wake_.reset();

    if (own_epfd_ && fds_.epfd >= 0) { ::close(fds_.epfd); fds_.epfd = -1; own_epfd_ = false; }
    if (own_efd_  && fds_.efd  >= 0) { ::close(fds_.efd ); fds_.efd  = -1; own_efd_  = false; }
    if (own_tfd_  && fds_.tfd  >= 0) { ::close(fds_.tfd ); fds_.tfd  = -1; own_tfd_  = false; }

    // 우리가 만든 소켓만 닫기
    if (own_sock_ && fds_.sock_meta >= 0) {
        ::close(fds_.sock_meta);
        fds_.sock_meta = -1;
        own_sock_ = false;
    }
}

void Meta_TxThread::set_meta_target(const struct sockaddr* sa, socklen_t slen) {
    ::memset(&sa_meta_, 0, sizeof(sa_meta_));
    ::memcpy(&sa_meta_, sa, slen); sl_meta_ = slen;
}

// ---- epoll 루프 ----
void Meta_TxThread::run() {
    constexpr int MAXE = 8;
    epoll_event evs[MAXE];

    while (running_.load()) {
        int n = ::epoll_wait(fds_.epfd, evs, MAXE, -1);
        if (n < 0) {
            if (errno == EINTR) continue;  // 신호로 깨어난 경우
            log_debug("epoll_wait err: " + std::string(strerror(errno)));
            break;
        }
        if (n == 0) continue;

        for (int i = 0; i < n; ++i) {
            int fd = evs[i].data.fd;
            if (fd == fds_.efd)      on_eventfd_ready();
            else if (fd == fds_.tfd) on_timerfd_ready();
        }
    }
}

// ---- 즉시 송신(코얼레스 없음) ----
void Meta_TxThread::on_eventfd_ready() {
    drain_eventfd(fds_.efd);

    while (auto ev = inbox_.exchange(nullptr)) {
        switch (ev->type) {
            case EventType::Track: {
                const auto& x = std::get<TrackEvent>(ev->payload);
                last_trk_.box = x.box; last_trk_.score = x.score; last_trk_.ts = x.ts; last_trk_.frame_seq = x.frame_seq;

                MetaTrackPacket p{};
                p.seq = x.frame_seq; p.ts = x.ts; // 이미 ns/epoch 등 상위에서 정한 단위
                p.x = x.box.x; p.y = x.box.y; p.w = x.box.width; p.h = x.box.height;
                p.score = x.score;
                send_track(p);
                break;
            }
            case EventType::Aruco: {
                const auto& x = std::get<ArucoEvent>(ev->payload);
                last_aru_.id = x.id; last_aru_.box = x.box; last_aru_.ts = x.ts;

                MetaArucoPacket p{};
                p.seq = 0; p.ts = x.ts; p.id = x.id;
                p.x = x.box.x; p.y = x.box.y; p.w = x.box.width; p.h = x.box.height;
                send_aruco(p);
                break;
            }
            case EventType::MetaCtrl: {
                const auto& x = std::get<MetaCtrlEvent>(ev->payload);
                last_ctl_.last_cmd = x.cmd; last_ctl_.ts = x.ts;

                MetaCtrlPacket p{}; p.seq = 0; p.ts = x.ts; p.state_or_cmd = x.cmd;
                send_ctrl(p);
                break;
            }
            case EventType::CtrlState: {
                const auto& x = std::get<CtrlStateEvent>(ev->payload);
                last_ctl_.state = x.state; last_ctl_.ts = x.ts;

                MetaCtrlPacket p{}; p.seq = 0; p.ts = x.ts; p.state_or_cmd = x.state;
                send_ctrl(p);
                break;
            }
            default:
                break;
        }
    }

    last_sent_ns_ = now_ns();
}

void Meta_TxThread::on_timerfd_ready() {
    drain_timerfd(fds_.tfd);
    auto hb = make_meta_hb();
    send_hb(hb);
}

// ---- 패킷 빌더(하트비트만 사용. 나머지는 즉시 구성/전송) ----
MetaTrackPacket Meta_TxThread::make_meta_track() const {
    MetaTrackPacket p{};
    p.seq = last_trk_.frame_seq; p.ts = last_trk_.ts;
    p.x = last_trk_.box.x; p.y = last_trk_.box.y; p.w = last_trk_.box.width; p.h = last_trk_.box.height;
    p.score = last_trk_.score;
    return p;
}
MetaArucoPacket Meta_TxThread::make_meta_aruco() const {
    MetaArucoPacket p{};
    p.seq = 0; p.ts = last_aru_.ts; p.id = last_aru_.id;
    p.x = last_aru_.box.x; p.y = last_aru_.box.y; p.w = last_aru_.box.width; p.h = last_aru_.box.height;
    return p;
}
MetaCtrlPacket Meta_TxThread::make_meta_ctrl() const {
    MetaCtrlPacket p{};
    p.seq = 0; p.ts = last_ctl_.ts;
    p.state_or_cmd = (last_ctl_.last_cmd ? last_ctl_.last_cmd : last_ctl_.state);
    return p;
}
MetaHBPacket Meta_TxThread::make_meta_hb() const {
    MetaHBPacket p{};
    p.ts = now_ms_epoch();               // ★ HB는 epoch ms로
    return p;
}

// ---- 송신(단일 소켓/목적지) ----
void Meta_TxThread::send_track(const MetaTrackPacket& p) {
    if (fds_.sock_meta >= 0 && sl_meta_ > 0) {
        auto buf = build_track(p.ts, p.seq, {p.x,p.y,p.w,p.h}, p.score);
        ssize_t n = ::sendto(fds_.sock_meta, buf.bytes.data(), buf.bytes.size(), 0,
                             (sockaddr*)&sa_meta_, sl_meta_);
        if (n < 0) log_debug("send_track failed: " + std::string(strerror(errno)));
    }
}
void Meta_TxThread::send_aruco(const MetaArucoPacket& p) {
    if (fds_.sock_meta >= 0 && sl_meta_ > 0) {
        auto buf = build_aruco(p.ts, p.id, {p.x,p.y,p.w,p.h});
        ssize_t n = ::sendto(fds_.sock_meta, buf.bytes.data(), buf.bytes.size(), 0,
                             (sockaddr*)&sa_meta_, sl_meta_);
        if (n < 0) log_debug("send_aruco failed: " + std::string(strerror(errno)));
    }
}
void Meta_TxThread::send_ctrl(const MetaCtrlPacket& p) {
    if (fds_.sock_meta >= 0 && sl_meta_ > 0) {
        auto buf = build_ctrl(p.ts, p.state_or_cmd);
        ssize_t n = ::sendto(fds_.sock_meta, buf.bytes.data(), buf.bytes.size(), 0,
                             (sockaddr*)&sa_meta_, sl_meta_);
        if (n < 0) log_debug("send_ctrl failed: " + std::string(strerror(errno)));
    }
}
void Meta_TxThread::send_hb(const MetaHBPacket& p) {
    if (fds_.sock_meta >= 0 && sl_meta_ > 0) {
        auto buf = build_hb(p.ts);
        ssize_t n = ::sendto(fds_.sock_meta, buf.bytes.data(), buf.bytes.size(), 0,
                             (sockaddr*)&sa_meta_, sl_meta_);
        if (n < 0) log_debug("send_hb failed: " + std::string(strerror(errno)));
    }
}

void Meta_TxThread::log_debug(const std::string& msg) {
    std::cout << "[Meta_TxThread] " << msg << std::endl;
}

} // namespace flir

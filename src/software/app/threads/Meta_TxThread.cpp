//Meta_TxThread.cpp
#include "threads_includes/Meta_TxThread.hpp"

// 프로토콜 빌더/이벤트 페이로드 타입은 cpp에서만 include
#include "components/includes/MetaWire.hpp"   // build_track/build_aruco/build_ctrl/build_hb
#include "ipc/ipc_types.hpp"                  // TrackEvent/ArucoEvent/MetaCtrlEvent/CtrlStateEvent

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <unistd.h>

namespace flir {

namespace {
constexpr const char* TAG = "Meta_Tx";

inline uint64_t now_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}
inline uint64_t now_ms_epoch() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
inline void drain_eventfd(int efd) {
    uint64_t cnt; while (::read(efd, &cnt, sizeof(cnt)) > 0) {}
}
inline void drain_timerfd(int tfd) {
    uint64_t expirations; (void)::read(tfd, &expirations, sizeof(expirations));
}
inline itimerspec make_period_ms(int ms) {
    itimerspec its{}; its.it_value.tv_sec = ms/1000; its.it_value.tv_nsec = (ms%1000)*1000000LL; its.it_interval = its.it_value; return its;
}
inline bool make_sockaddr_ipv4(const char* ip, uint16_t port, sockaddr_storage& ss, socklen_t& sl) {
    sockaddr_in sa{}; sa.sin_family = AF_INET; sa.sin_port = htons(port);
    if (::inet_pton(AF_INET, ip, &sa.sin_addr) != 1) return false;
    std::memset(&ss, 0, sizeof(ss)); std::memcpy(&ss, &sa, sizeof(sa)); sl = sizeof(sa); return true;
}
} // namespace

// eventfd 기반 wake 핸들 (내부 정의)
class Meta_TxThread::EfdWakeHandle : public WakeHandle {
public:
    explicit EfdWakeHandle(int efd) : efd_(efd) {}
    void signal() override { uint64_t one = 1; (void)!::write(efd_, &one, sizeof(one)); }
private:
    int efd_;
};

Meta_TxThread::Meta_TxThread(IEventBus& bus, AppConfigPtr cfg)
    : bus_(bus), cfg_(std::move(cfg)) {}
Meta_TxThread::~Meta_TxThread() { stop(); join(); close_io_(); }

void Meta_TxThread::start() {
    if (running_.exchange(true)) return;
    if (!init_io_()) { running_.store(false); throw std::runtime_error("Meta_TxThread init_io failed"); }

    wake_ = std::make_unique<EfdWakeHandle>(efd_);
    bus_.subscribe(Topic::Tracking, &inbox_, wake_.get());
    bus_.subscribe(Topic::Aruco,    &inbox_, wake_.get());
    bus_.subscribe(Topic::Control,  &inbox_, wake_.get());

    hb_period_ms_ = cfg_->meta_tx.hb_period_ms;
    th_ = std::thread(&Meta_TxThread::run_, this);
}

void Meta_TxThread::stop() {
    if (!running_.load()) return;
    running_.store(false);
    if (efd_ >= 0) { EfdWakeHandle tmp(efd_); tmp.signal(); }
}
void Meta_TxThread::join() {
    if (th_.joinable()) th_.join();
    bus_.unsubscribe(&inbox_);
    wake_.reset();
}

bool Meta_TxThread::init_io_() {
    close_io_(); // 방어적

    epfd_ = ::epoll_create1(EPOLL_CLOEXEC);
    if (epfd_ < 0) { LOGE(TAG, "epoll_create1: %s", strerror(errno)); return false; }
    own_epfd_ = true;

    efd_ = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (efd_ < 0) { LOGE(TAG, "eventfd: %s", strerror(errno)); return false; }
    own_efd_ = true;

    tfd_ = ::timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
    if (tfd_ < 0) { LOGE(TAG, "timerfd_create: %s", strerror(errno)); return false; }
    own_tfd_ = true;

    itimerspec its = make_period_ms(cfg_->meta_tx.hb_period_ms);
    ::timerfd_settime(tfd_, 0, &its, nullptr);

    epoll_event ev{}; ev.events = EPOLLIN;
    ev.data.fd = efd_; (void)::epoll_ctl(epfd_, EPOLL_CTL_ADD, efd_, &ev);
    ev.data.fd = tfd_; (void)::epoll_ctl(epfd_, EPOLL_CTL_ADD, tfd_, &ev);

    sock_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (sock_ < 0) { LOGE(TAG, "socket: %s", strerror(errno)); return false; }
    own_sock_ = true;

    if (cfg_->meta_tx.sndbuf_bytes > 0) {
        int sz = cfg_->meta_tx.sndbuf_bytes;
        if (::setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, &sz, sizeof(sz)) != 0) {
            LOGW(TAG, "setsockopt(SO_SNDBUF=%d) failed: %s", sz, strerror(errno));
        }
    }
    if (cfg_->meta_tx.local_port != 0) {
        sockaddr_in a{}; a.sin_family = AF_INET; a.sin_addr.s_addr = htonl(INADDR_ANY); a.sin_port = htons(cfg_->meta_tx.local_port);
        if (::bind(sock_, (sockaddr*)&a, sizeof(a)) != 0) LOGE(TAG, "bind :%u failed: %s", cfg_->meta_tx.local_port, strerror(errno));
        else LOGI(TAG, "bound on :%u", cfg_->meta_tx.local_port);
    }
    if (!cfg_->meta_tx.dst.ip.empty() && cfg_->meta_tx.dst.port != 0) {
        sockaddr_storage dst{}; socklen_t dlen = 0;
        if (make_sockaddr_ipv4(cfg_->meta_tx.dst.ip.c_str(), cfg_->meta_tx.dst.port, dst, dlen)) {
            set_meta_target_((sockaddr*)&dst, dlen);
            LOGI(TAG, "target %s:%u", cfg_->meta_tx.dst.ip.c_str(), cfg_->meta_tx.dst.port);
        } else {
            LOGE(TAG, "invalid remote_ip: %s", cfg_->meta_tx.dst.ip.c_str());
        }
    } else {
        LOGW(TAG, "remote target not set");
    }
    return true;
}

void Meta_TxThread::close_io_() {
    if (own_epfd_ && epfd_ >= 0) { ::close(epfd_); epfd_ = -1; own_epfd_ = false; }
    if (own_efd_  && efd_  >= 0) { ::close(efd_ ); efd_  = -1; own_efd_  = false; }
    if (own_tfd_  && tfd_  >= 0) { ::close(tfd_ ); tfd_  = -1; own_tfd_  = false; }
    if (own_sock_ && sock_ >= 0) { ::close(sock_); sock_ = -1; own_sock_ = false; }
}
void Meta_TxThread::set_meta_target_(const struct sockaddr* sa, socklen_t slen) {
    std::memset(&sa_meta_, 0, sizeof(sa_meta_));
    std::memcpy(&sa_meta_, sa, slen);
    sl_meta_ = slen;
}

void Meta_TxThread::run_() {
    constexpr int MAXE = 8; epoll_event evs[MAXE];
    while (running_.load()) {
        int n = ::epoll_wait(epfd_, evs, MAXE, -1);
        if (n < 0) { if (errno == EINTR) continue; LOGE(TAG, "epoll_wait: %s", strerror(errno)); break; }
        if (n == 0) continue;
        for (int i = 0; i < n; ++i) {
            int fd = evs[i].data.fd;
            if (fd == efd_)      on_eventfd_ready_();
            else if (fd == tfd_) on_timerfd_ready_();
        }
    }
    LOGI(TAG, "run() exit");
}

void Meta_TxThread::send_aruco_full_(uint64_t ts, int id,
                                     const cv::Rect2f& box,
                                     const std::array<cv::Point2f,4>& c) {
    if (sock_ < 0 || sl_meta_ == 0) return;
    auto buf = build_aruco_full(ts, id, box, c);     // ★ 코너 포함
    ssize_t n = ::sendto(sock_, buf.bytes.data(), buf.bytes.size(), 0,
                         (sockaddr*)&sa_meta_, sl_meta_);
    if (n < 0) LOGE(TAG, "send_aruco(full) failed: %s", strerror(errno));
    else LOGDs(TAG) << "TX ARUCO(full) bytes=" << n;
}

// (옵션) 하위호환: bbox-only 가 필요하면 이 함수도 남겨둠
void Meta_TxThread::send_aruco_bbox_(uint64_t ts, int id, const cv::Rect2f& box) {
    if (sock_ < 0 || sl_meta_ == 0) return;
    auto buf = build_aruco(ts, id, box);
    (void)::sendto(sock_, buf.bytes.data(), buf.bytes.size(), 0,
                   (sockaddr*)&sa_meta_, sl_meta_);
}


void Meta_TxThread::on_eventfd_ready_() {
    drain_eventfd(efd_);
    while (auto ev = inbox_.exchange(nullptr)) {
        switch (ev->type) {
            case EventType::Track: {
                const auto& x = std::get<TrackEvent>(ev->payload);
                last_trk_ = { (float)x.box.x, (float)x.box.y, (float)x.box.width, (float)x.box.height, x.score, x.ts, x.frame_seq };
                send_track_(x.ts, x.frame_seq, (float)x.box.x, (float)x.box.y, (float)x.box.width, (float)x.box.height, x.score);
                break;
            }
            case EventType::Aruco: {
                const auto& x = std::get<ArucoEvent>(ev->payload);
                last_aru_ = { x.id, (float)x.box.x, (float)x.box.y, (float)x.box.width, (float)x.box.height, x.ts };
                // ✅ 코너까지 포함해서 전송
                send_aruco_full_(x.ts, x.id, x.box, x.corners);
                break;
            }
            case EventType::MetaCtrl: {
                const auto& x = std::get<MetaCtrlEvent>(ev->payload);
                last_ctl_.last_cmd = x.cmd; last_ctl_.ts = x.ts;
                send_ctrl_(x.ts, x.cmd);
                break;
            }
            case EventType::CtrlState: {
                const auto& x = std::get<CtrlStateEvent>(ev->payload);
                last_ctl_.state = x.state; last_ctl_.ts = x.ts;
                send_ctrl_(x.ts, x.state);
                break;
            }
            default: break;
        }
    }
    last_sent_ns_ = now_ns();
}

void Meta_TxThread::on_timerfd_ready_() {
    drain_timerfd(tfd_);
    send_hb_(now_ms_epoch());  // HB는 epoch ms
}

// ===== 송신 (cpp에서만 MetaWire 사용) =====
void Meta_TxThread::send_track_(uint64_t ts, uint32_t seq, float x, float y, float w, float h, float score) {
    if (sock_ < 0 || sl_meta_ == 0) return;
    auto buf = build_track(ts, seq, {x,y,w,h}, score);
    ssize_t n = ::sendto(sock_, buf.bytes.data(), buf.bytes.size(), 0, (sockaddr*)&sa_meta_, sl_meta_);
    if (n < 0) LOGE(TAG, "send_track failed: %s", strerror(errno)); else LOGDs(TAG) << "TX TRACK bytes=" << n;
}
void Meta_TxThread::send_aruco_(uint64_t ts, int id, float x, float y, float w, float h) {
    if (sock_ < 0 || sl_meta_ == 0) return;
    auto buf = build_aruco(ts, id, {x,y,w,h});
    ssize_t n = ::sendto(sock_, buf.bytes.data(), buf.bytes.size(), 0, (sockaddr*)&sa_meta_, sl_meta_);
    if (n < 0) LOGE(TAG, "send_aruco failed: %s", strerror(errno)); else LOGDs(TAG) << "TX ARUCO bytes=" << n;
}
void Meta_TxThread::send_ctrl_(uint64_t ts, uint32_t state_or_cmd) {
    if (sock_ < 0 || sl_meta_ == 0) return;
    auto buf = build_ctrl(ts, state_or_cmd);
    ssize_t n = ::sendto(sock_, buf.bytes.data(), buf.bytes.size(), 0, (sockaddr*)&sa_meta_, sl_meta_);
    if (n < 0) LOGE(TAG, "send_ctrl failed: %s", strerror(errno)); else LOGDs(TAG) << "TX CTRL bytes=" << n;
}
void Meta_TxThread::send_hb_(uint64_t ts) {
    if (sock_ < 0 || sl_meta_ == 0) return;
    auto buf = build_hb(ts);
    ssize_t n = ::sendto(sock_, buf.bytes.data(), buf.bytes.size(), 0, (sockaddr*)&sa_meta_, sl_meta_);
    if (n < 0) LOGE(TAG, "send_hb failed: %s", strerror(errno)); else LOGDs(TAG) << "TX HB bytes=" << n;
}
} // namespace flir

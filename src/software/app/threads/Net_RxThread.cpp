#include "threads_includes/Net_RxThread.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <sstream>
#include <string_view>
#include <opencv2/core.hpp>   // cv::Rect2f

namespace flir {

namespace { constexpr const char* TAG = "Net_Rx"; }

Net_RxThread::Net_RxThread(std::string name,
                           AppConfigPtr cfg,
                           SpscMailbox<UserCmd>& outbox)
    : name_(std::move(name)), cfg_(std::move(cfg)), outbox_(outbox) {
    rxbuf_.resize(std::max<size_t>(cfg_->net_rx.buffer_size, 1024));
}

Net_RxThread::~Net_RxThread() {
    stop();
    join();
    close_socket_();
}

void Net_RxThread::start() {
    if (running_.exchange(true)) return;
    if (!init_socket_()) { running_.store(false); throw std::runtime_error("Net_RxThread socket init failed"); }
    th_ = std::thread(&Net_RxThread::run_, this);
}

void Net_RxThread::stop() {
    if (!running_.load()) return;
    running_.store(false);
    // poll 깨우기용 dummy datagram (loopback)
    if (sock_ >= 0) {
        sockaddr_in self{}; self.sin_family = AF_INET; self.sin_port = htons(bind_port_); self.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        ::sendto(sock_, "", 0, 0, (sockaddr*)&self, sizeof(self));
    }
}

void Net_RxThread::join() {
    if (th_.joinable()) th_.join();
}

bool Net_RxThread::init_socket_() {
    close_socket_();

    bind_port_ = cfg_->net_rx.port;
    sock_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (sock_ < 0) { LOGE(TAG, "socket: %s", strerror(errno)); return false; }
    own_sock_ = true;

    sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons(bind_port_); a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(sock_, (sockaddr*)&a, sizeof(a)) != 0) {
        LOGE(TAG, "bind :%u failed: %s", bind_port_, strerror(errno));
        ::close(sock_); sock_ = -1; own_sock_ = false; return false;
    }

    LOGI(TAG, "listening UDP :%u (timeout=%dms, buf=%zu, click_box=%.1f)",
         bind_port_, cfg_->net_rx.timeout_ms, rxbuf_.size(), cfg_->net_rx.click_box_size);
    return true;
}

void Net_RxThread::close_socket_() {
    if (own_sock_ && sock_ >= 0) { ::close(sock_); sock_ = -1; own_sock_ = false; }
}

void Net_RxThread::run_() {
    const int timeout_ms = cfg_->net_rx.timeout_ms;

    while (running_.load()) {
        pollfd pfd{ .fd = sock_, .events = POLLIN, .revents = 0 };
        int r = ::poll(&pfd, 1, timeout_ms);
        if (!running_.load()) break;

        if (r < 0) {
            if (errno == EINTR) continue;
            LOGE(TAG, "poll: %s", strerror(errno));
            break;
        }
        if (r == 0) continue;

        if (pfd.revents & POLLIN) {
            sockaddr_in src{}; socklen_t sl = sizeof(src);
            const ssize_t n = ::recvfrom(sock_, rxbuf_.data(), rxbuf_.size(), 0, (sockaddr*)&src, &sl);
            if (n <= 0) continue;

            handle_datagram_(rxbuf_.data(), static_cast<size_t>(n), src);
        }
    }

    LOGI(TAG, "run() exit");
}

void Net_RxThread::handle_datagram_(const uint8_t* data, size_t len, const sockaddr_in& src) {
    if (len == 0) return; // 빈 패킷 무시

    // 1) PC 바이너리 포맷: 0 | f32(x)be | f32(y)be
    if (len == 9 && data[0] == 0) {
        int32_t xi_be, yi_be;
        std::memcpy(&xi_be, data+1, 4);
        std::memcpy(&yi_be, data+5, 4);
        int32_t xi = ntohl(static_cast<uint32_t>(xi_be));
        int32_t yi = ntohl(static_cast<uint32_t>(yi_be));

        float fx; std::memcpy(&fx, &xi, sizeof(float));
        float fy; std::memcpy(&fy, &yi, sizeof(float));

        if (std::isfinite(fx) && std::isfinite(fy)) {
            const float b = cfg_->net_rx.click_box_size;
            UserCmd cmd{};
            cmd.type = CmdType::CLICK;
            cmd.box  = cv::Rect2f(fx - b*0.5f, fy - b*0.5f, b, b); // 이미 픽셀좌표로 전송됨
            cmd.seq  = ++cmd_seq_;
            outbox_.push(std::move(cmd));
            LOGI(TAG, "CLICK(bin C#) x=%.1f y=%.1f box=%.1f", fx, fy, b);
            return;
        }
        // 형식은 맞았는데 값이 비정상이면 아래 텍스트 경로로 그냥 떨어짐
    }

    // 2) 텍스트 형식(기존): "CLICK x y" / "x y" / "x,y"
    std::string_view sv(reinterpret_cast<const char*>(data), len);
    std::string msg(sv);
    // trim
    auto issp = [](unsigned char c){ return std::isspace(c); };
    msg.erase(msg.begin(), std::find_if(msg.begin(), msg.end(), [&](char c){ return !issp(c); }));
    msg.erase(std::find_if(msg.rbegin(), msg.rend(), [&](char c){ return !issp(c); }).base(), msg.end());

    if (!msg.empty()) {
        UserCmd cmd{};
        if (parse_cmd_click_(msg, cmd)) {
            outbox_.push(std::move(cmd));
            return;
        }
    }

    // 3) 그 외는 조용히 드롭(스팸 방지)
    LOGW(TAG, "unrecognized msg ...");
}


bool Net_RxThread::parse_cmd_click_(const std::string& msg, UserCmd& out) {
    // "CLICK x y" (x,y: 픽셀 또는 정규화; 팀 규약에 맞게 사용)
    std::istringstream iss(msg);
    std::string head;
    if (!(iss >> head)) return false;
    if (!(head == "CLICK" || head == "click")) return false;

    double vx = 0.0, vy = 0.0;
    if (!(iss >> vx >> vy)) return false;

    const float x = static_cast<float>(vx);
    const float y = static_cast<float>(vy);
    const float b = cfg_->net_rx.click_box_size;

    // UserCmd: { CmdType type; cv::Rect2f box; uint32_t seq; }
    out.type = CmdType::CLICK;
    out.box  = cv::Rect2f(x - b*0.5f, y - b*0.5f, b, b);
    out.seq  = ++cmd_seq_;                 // 📌 Mailbox가 latest_seq로 쓸 값

    LOGI(TAG, "CLICK x=%.1f y=%.1f box=%.1f (seq=%u)", x, y, b, out.seq);
    return true;
}

} // namespace flir

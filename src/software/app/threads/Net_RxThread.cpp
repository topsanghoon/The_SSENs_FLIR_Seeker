#include "threads_includes/Net_RxThread.hpp"
#include "components/includes/common_log.hpp"

#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

namespace flir {

static constexpr const char* TAG = "Net_RxThread";

Net_RxThread::Net_RxThread(SpscMailbox<UserCmd>& cmd_mb, NetRxConfig cfg)
    : cmd_mb_(cmd_mb), cfg_(std::move(cfg)) {
}

Net_RxThread::~Net_RxThread() {
    if (running_.load()) {
        stop();
        join();
    }
}

void Net_RxThread::start() {
    if (running_.exchange(true)) {
        LOGDs(TAG) << "already running";
        return;
    }
    th_ = std::thread(&Net_RxThread::run, this);
    LOGDs(TAG) << "started";
}

void Net_RxThread::stop() {
    if (!running_.exchange(false)) return;
    if (socket_fd_ != -1) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    LOGDs(TAG) << "stop requested";
}

void Net_RxThread::join() {
    if (th_.joinable()) {
        th_.join();
        LOGDs(TAG) << "joined";
    }
}

void Net_RxThread::run() {
    LOGDs(TAG) << "run() starting";
    try {
        setup_socket();
        LOGDs(TAG) << "listening UDP port " << cfg_.port;

        std::vector<uint8_t> buffer(cfg_.buffer_size);
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);

        while (running_.load()) {
            ssize_t n = recvfrom(socket_fd_, buffer.data(), buffer.size(), 0,
                                 (struct sockaddr*)&client_addr, &client_len);
            if (n > 0) {
                received_count_.fetch_add(1);

                char ip[INET_ADDRSTRLEN] = {0};
                inet_ntop(AF_INET, &client_addr.sin_addr, ip, INET_ADDRSTRLEN);
                LOGD(TAG, "recv %zd bytes from %s:%u", n, ip, ntohs(client_addr.sin_port));

                UserCmd cmd;
                if (parse_user_command(buffer.data(), (size_t)n, cmd)) {
                    cmd_mb_.push(std::move(cmd));
                    processed_count_.fetch_add(1);
                    LOGDs(TAG) << "pushed UserCmd seq=" << cmd.seq
                               << " box=(" << cmd.box.x << "," << cmd.box.y << ","
                               << cmd.box.width << "," << cmd.box.height << ")";
                }
            } else if (n == -1) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue; // timeout
                } else if (errno == EBADF || errno == ENOTSOCK) {
                    break;    // socket closed (normal)
                } else {
                    LOGEs(TAG) << "recvfrom error: " << strerror(errno);
                    break;
                }
            }
        }
        LOGDs(TAG) << "receive loop exited";
    } catch (const std::exception& e) {
        LOGEs(TAG) << "exception in run(): " << e.what();
    }
    cleanup_socket();
    LOGDs(TAG) << "run() finished";
}

void Net_RxThread::setup_socket() {
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ == -1) {
        throw std::runtime_error(std::string("socket() failed: ") + strerror(errno));
    }

    // recv timeout
    timeval tv{};
    tv.tv_sec  = cfg_.timeout_ms / 1000;
    tv.tv_usec = (cfg_.timeout_ms % 1000) * 1000;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        close(socket_fd_); socket_fd_ = -1;
        throw std::runtime_error(std::string("setsockopt(SO_RCVTIMEO) failed: ") + strerror(errno));
    }

    int reuse = 1;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        close(socket_fd_); socket_fd_ = -1;
        throw std::runtime_error(std::string("setsockopt(SO_REUSEADDR) failed: ") + strerror(errno));
    }

    std::memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_addr.s_addr = INADDR_ANY;
    server_addr_.sin_port = htons(cfg_.port);
    if (bind(socket_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) == -1) {
        close(socket_fd_); socket_fd_ = -1;
        throw std::runtime_error("bind() failed on port " + std::to_string(cfg_.port) +
                                 ": " + std::string(strerror(errno)));
    }
    LOGDs(TAG) << "socket ready on port " << cfg_.port;
}

void Net_RxThread::cleanup_socket() {
    if (socket_fd_ != -1) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    LOGDs(TAG) << "socket cleanup";
}

// Binary: [type:1][x:float(be):4][y:float(be):4] => 9 bytes
bool Net_RxThread::parse_user_command(const uint8_t* data, size_t size, UserCmd& cmd) {
    try {
        if (size >= 9 && data[0] == 0) { // CLICK
            union { uint32_t i; float f; } u;

            u.i = ntohl(*reinterpret_cast<const uint32_t*>(&data[1]));
            float x = u.f;

            u.i = ntohl(*reinterpret_cast<const uint32_t*>(&data[5]));
            float y = u.f;

            if (x >= 0 && y >= 0 && x <= 4096 && y <= 4096) {
                float s  = cfg_.click_box_size;
                float bx = std::max(0.0f, x - s * 0.5f);
                float by = std::max(0.0f, y - s * 0.5f);

                cmd.type = CmdType::CLICK;
                cmd.box  = cv::Rect2f(bx, by, s, s);
                cmd.seq  = cmd_seq_.fetch_add(1);

                LOGDs(TAG) << "parsed BIN CLICK x=" << x << " y=" << y
                           << " -> box=(" << bx << "," << by << "," << s << "," << s << ")";
                return true;
            } else {
                LOGWs(TAG) << "BIN CLICK out-of-range x=" << x << " y=" << y;
            }
        } else {
            LOGWs(TAG) << "not a BIN CLICK packet (len=" << size << ", type=" << int(data[0]) << ")";
        }
    } catch (const std::exception& e) {
        LOGEs(TAG) << "exception parsing user command: " << e.what();
    }
    return false;
}

} // namespace flir

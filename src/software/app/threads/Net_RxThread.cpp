#include "threads_includes/Net_RxThread.hpp"
#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

namespace flir {

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
        return;
    }
    
    th_ = std::thread(&Net_RxThread::run, this);
}

void Net_RxThread::stop() {
    if (!running_.exchange(false)) {
        return;
    }
    
    // Close socket to break out of recv calls
    if (socket_fd_ != -1) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

void Net_RxThread::join() {
    if (th_.joinable()) {
        th_.join();
    }
}

void Net_RxThread::run() {
    log_debug("Net_RxThread::run() starting");
    
    try {
        setup_socket();
        
        log_debug("UDP socket listening on port " + std::to_string(config_.port));
        
        // Main receive loop
        std::vector<uint8_t> buffer(config_.buffer_size);
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        while (running_.load()) {
            // Receive data from Windows PC
            ssize_t bytes_received = recvfrom(socket_fd_, buffer.data(), buffer.size(), 0,
                                            (struct sockaddr*)&client_addr, &client_len);
            
            if (bytes_received > 0) {
                received_count_.fetch_add(1);
                
                if (config_.enable_debug) {
                    char client_ip[INET_ADDRSTRLEN];
                    inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
                    log_debug("Received " + std::to_string(bytes_received) + 
                             " bytes from " + std::string(client_ip) + ":" + 
                             std::to_string(ntohs(client_addr.sin_port)));
                }
                
                // Try to parse user command
                UserCmd cmd;
                if (parse_user_command(buffer.data(), bytes_received, cmd)) {
                    // Send command to IR_TrackThread via mailbox
                    cmd_mb_.push(std::move(cmd));
                    processed_count_.fetch_add(1);
                    
                    if (config_.enable_debug) {
                        log_debug("Processed UserCmd: type=" + std::to_string(static_cast<int>(cmd.type)) +
                                 " box=(" + std::to_string(cmd.box.x) + "," + std::to_string(cmd.box.y) + "," +
                                 std::to_string(cmd.box.width) + "," + std::to_string(cmd.box.height) + ")" +
                                 " seq=" + std::to_string(cmd.seq));
                    }
                }
            } else if (bytes_received == -1) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    // Timeout, continue loop
                    continue;
                } else if (errno == EBADF || errno == ENOTSOCK) {
                    // Socket closed (normal shutdown)
                    break;
                } else {
                    log_debug("recvfrom error: " + std::string(strerror(errno)));
                    break;
                }
            }
        }
        
        log_debug("Receive loop exited");
        
    } catch (const std::exception& e) {
        log_debug(std::string("Exception in Net_RxThread::run(): ") + e.what());
    }
    
    cleanup_socket();
    log_debug("Net_RxThread::run() finished");
}

void Net_RxThread::setup_socket() {
    // Create UDP socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ == -1) {
        throw std::runtime_error("Failed to create socket: " + std::string(strerror(errno)));
    }
    
    // Set socket to non-blocking with timeout
    struct timeval timeout;
    timeout.tv_sec = config_.timeout_ms / 1000;
    timeout.tv_usec = (config_.timeout_ms % 1000) * 1000;
    
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        throw std::runtime_error("Failed to set socket timeout: " + std::string(strerror(errno)));
    }
    
    // Allow socket reuse
    int reuse = 1;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        close(socket_fd_);
        socket_fd_ = -1;
        throw std::runtime_error("Failed to set socket reuse: " + std::string(strerror(errno)));
    }
    
    // Configure server address
    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_addr.s_addr = INADDR_ANY;
    server_addr_.sin_port = htons(config_.port);
    
    // Bind socket
    if (bind(socket_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) == -1) {
        close(socket_fd_);
        socket_fd_ = -1;
        throw std::runtime_error("Failed to bind socket to port " + std::to_string(config_.port) + 
                                ": " + std::string(strerror(errno)));
    }
    
    log_debug("UDP socket set up successfully on port " + std::to_string(config_.port));
}

void Net_RxThread::cleanup_socket() {
    if (socket_fd_ != -1) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    log_debug("Socket cleanup completed");
}

bool Net_RxThread::parse_user_command(const uint8_t* data, size_t size, UserCmd& cmd) {
    try {
        // Binary: [type:1][x:float(be):4][y:float(be):4]  => 총 9 바이트
        if (size >= 9 && data[0] == 0) {
            union { uint32_t i; float f; } u;
            u.i = ntohl(*reinterpret_cast<const uint32_t*>(&data[1]));
            float x = u.f;
            u.i = ntohl(*reinterpret_cast<const uint32_t*>(&data[5]));
            float y = u.f;

            if (x >= 0 && y >= 0 && x <= 4096 && y <= 4096) {
                float s  = config_.click_box_size;
                float bx = std::max(0.0f, x - s * 0.5f);
                float by = std::max(0.0f, y - s * 0.5f);

                cmd.type = CmdType::CLICK;
                cmd.box  = cv::Rect2f(bx, by, s, s);
                cmd.seq  = cmd_seq_.fetch_add(1);

                if (config_.enable_debug) {
                    log_debug("Parsed BIN CLICK x=" + std::to_string(x) + " y=" + std::to_string(y));
                }
                return true;
            } else {
                if (config_.enable_debug) log_debug("BIN CLICK out-of-range");
            }
        } else {
            if (config_.enable_debug) log_debug("Not a BIN CLICK packet (len=" + std::to_string(size) + ")");
        }
    } catch (const std::exception& e) {
        log_debug(std::string("Exception parsing user command: ") + e.what());
    }
    return false;
}


void Net_RxThread::log_debug(const std::string& msg) {
    if (config_.enable_debug) {
        std::cout << "[Net_RxThread] " << msg << std::endl;
    }
}

} // namespace flir
#pragma once
#include <atomic>
#include <thread>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>

#include "ipc/ipc_types.hpp"
#include "ipc/mailbox.hpp"

namespace flir {

// Configuration for Net_RxThread
struct NetRxConfig {
    int port = 5000;
    bool enable_debug = false;
    int buffer_size = 1024;
    int timeout_ms = 100; // Socket timeout for non-blocking behavior
    float click_box_size = 40.0f; // Size of bounding box created around click point
};

// Network receive thread that gets user input from Windows PC via UDP
// Receives click commands and forwards to IR_TrackThread via mailbox
class Net_RxThread {
public:
    Net_RxThread(SpscMailbox<UserCmd>& cmd_mb, 
                 NetRxConfig cfg = {});
    
    ~Net_RxThread();

    void start();
    void stop();
    void join();

    // Stats/debug
    uint32_t get_received_count() const { return received_count_.load(); }
    uint32_t get_processed_count() const { return processed_count_.load(); }

private:
    // Configuration and mailbox
    SpscMailbox<UserCmd>& cmd_mb_;
    NetRxConfig cfg_;
    
    // Thread management
    std::thread th_;
    std::atomic<bool> running_{false};
    
    // Network components
    int socket_fd_ = -1;
    struct sockaddr_in server_addr_;
    
    // Statistics
    std::atomic<uint32_t> received_count_{0};
    std::atomic<uint32_t> processed_count_{0};
    std::atomic<uint32_t> cmd_seq_{1};
    
    // Internal methods
    void run();
    void setup_socket();
    void cleanup_socket();
    
    // Data processing
    bool parse_user_command(const uint8_t* data, size_t size, UserCmd& cmd);
    void log_debug(const std::string& msg);
};

} // namespace flir

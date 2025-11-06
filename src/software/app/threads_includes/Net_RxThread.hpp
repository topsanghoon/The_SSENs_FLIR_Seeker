#pragma once
#include <atomic>
#include <string>
#include <thread>
#include <vector>
#include <netinet/in.h>

#include "main_config.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/ipc_types.hpp"   // UserCmd, SelfDestructCmd
#include "ipc/event_bus.hpp"

namespace flir {

class Net_RxThread {
public:
    Net_RxThread(std::string name,
                 AppConfigPtr cfg,
                 SpscMailbox<UserCmd>& click_out,
                 SpscMailbox<SelfDestructCmd>& sd_out,
                IEventBus& bus);
    ~Net_RxThread();

    void start();
    void stop();
    void join();

private:
    bool init_socket_();
    void close_socket_();
    void run_();

    void handle_datagram_(const uint8_t* data, size_t len, const sockaddr_in& src);

    // 파서들
    bool parse_cmd_click_(const std::string& msg, UserCmd& out);
    bool parse_cmd_sd_text_(const std::string& msg, SelfDestructCmd& out);
    bool parse_cmd_sd_bin_(const uint8_t* data, size_t len, SelfDestructCmd& out);

private:
    std::string   name_;
    AppConfigPtr  cfg_;

    // 출력 메일박스
    SpscMailbox<UserCmd>&          out_click_;
    SpscMailbox<SelfDestructCmd>&  out_sd_;

    // 소켓
    int  sock_{-1};
    bool own_sock_{false};
    uint16_t bind_port_{0};

    // 버퍼
    std::vector<uint8_t> rxbuf_;

    // 스레드
    std::thread th_;
    std::atomic<bool> running_{false};

    // 시퀀스
    std::atomic<uint32_t> click_seq_{0};
    std::atomic<uint32_t> sd_seq_{0};

    IEventBus& bus_; 
};

} // namespace flir

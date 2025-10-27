#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <netinet/in.h>

#include "main_config.hpp"
#include "util/common_log.hpp"
#include "ipc/mailbox.hpp"
#include "ipc/ipc_types.hpp"   // CmdType, UserCmd

namespace flir {

class Net_RxThread {
public:
    Net_RxThread(std::string name,
                 AppConfigPtr cfg,
                 SpscMailbox<UserCmd>& outbox);
    ~Net_RxThread();

    Net_RxThread(const Net_RxThread&) = delete;
    Net_RxThread& operator=(const Net_RxThread&) = delete;

    void start();
    void stop();
    void join();

private:
    bool init_socket_();
    void close_socket_();
    void run_();
    void handle_datagram_(const uint8_t* data, size_t len, const sockaddr_in& src);

    // "CLICK x y" 텍스트 파서 → UserCmd로 채움
    bool parse_cmd_click_(const std::string& msg, UserCmd& out);

private:
    std::string name_;
    AppConfigPtr cfg_;
    SpscMailbox<UserCmd>& outbox_;

    std::thread       th_;
    std::atomic<bool> running_{false};

    int      sock_{-1};
    bool     own_sock_{false};
    uint16_t bind_port_{0};

    std::vector<uint8_t> rxbuf_;

    // UserCmd.seq 부여용 로컬 시퀀스
    uint32_t cmd_seq_{0};
};

} // namespace flir

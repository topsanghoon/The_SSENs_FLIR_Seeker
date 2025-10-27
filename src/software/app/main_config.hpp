#pragma once
#include <memory>
#include <string>
#include <cstdint>

namespace flir {

struct Endpoint { std::string ip; uint16_t port{}; };
struct VideoSize { int width{}; int height{}; };

struct PathsConfig {
    std::string csv_root = "./logs";  // ✅ 상대경로
    std::string tmp_root = "./tmp";
};

struct IRTxConfig {
    VideoSize frame{160,120};
    int       fps{30};
    int       bitDepth{16};           // gray16-le 등
    Endpoint  dst;                    // udp 목적지
};

struct EOTxConfig {
    VideoSize frame{640,480};
    int       fps{30};
    int       jpeg_quality{30};
    Endpoint  dst;
};

struct MetaTxConfigDI {
    Endpoint  dst;
    uint16_t  local_port{0};
    int       hb_period_ms{1000};
    int       sndbuf_bytes{1<<20};
};

struct NetRxConfigDI {
    uint16_t port{6002};
    int      timeout_ms{200};
    size_t   buffer_size{1024};
    float    click_box_size{64.0f};
};

struct AppConfig {
    PathsConfig     paths;
    IRTxConfig      ir_tx;
    EOTxConfig      eo_tx;
    MetaTxConfigDI  meta_tx;
    NetRxConfigDI   net_rx;
};

using AppConfigPtr = std::shared_ptr<const AppConfig>;

} // namespace flir

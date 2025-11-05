#pragma once
#include <memory>
#include <string>
#include <cstdint>
#include "guidance_mode.hpp"

namespace flir {

struct Endpoint { std::string ip; uint16_t port{}; };
struct VideoSize { int width{}; int height{}; };

struct PathsConfig {
    std::string csv_root = "./logs";  // ✅ 상대경로
    std::string tmp_root = "./tmp";
};

struct IRTxConfig {
    VideoSize frame{160,120};
    int       fps{15};
    int       bitDepth{16};           // gray16-le 등
    Endpoint  dst;                    // udp 목적지
};

struct EOTxConfig {
    VideoSize frame{320,240};
    int       fps{15};
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

struct GuidanceConfig {
    GuidancePhase default_phase{GuidancePhase::Midcourse};
    int  terminal_marker_id{3};  // 종말 전환 대상 마커 ID
    // 절대 크기(px) 기준
    int  min_bbox_w{160};
    int  min_bbox_h{160};
    int  min_big_frames{5};      // “충분히 큼” 연속 프레임 수
    int  hold_big_ms{300};   // 큰 상태 이후 미검출 지속 시간
    // (선택) 프레임 비율 기준: max(bw/W,bh/H) >= min_bbox_frac
    float min_bbox_frac{0.0f};   // 0이면 비활성
};

struct AppConfig {
    PathsConfig     paths;
    IRTxConfig      ir_tx;
    EOTxConfig      eo_tx;
    MetaTxConfigDI  meta_tx;
    NetRxConfigDI   net_rx;
    GuidanceConfig  guidance;
};

using AppConfigPtr = std::shared_ptr<const AppConfig>;

} // namespace flir

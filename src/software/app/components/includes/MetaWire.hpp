#pragma once
#include <cstdint>
#include <vector>
#include <array>
#include <opencv2/core.hpp>

namespace flir {

enum class MetaMsgType : uint8_t { Track=1, Aruco=2, Ctrl=3, Heartbeat=4 };
constexpr uint8_t META_WIRE_VERSION = 1;

// CPP가 기대하는 버퍼 형태: 멤버 bytes 보유
struct MetaBuffer {
    std::vector<uint8_t> bytes;
};

// 헤더는 packing 1바이트 정렬 유지
#pragma pack(push, 1)
struct MetaMsgHeader {
    uint8_t  version{META_WIRE_VERSION};
    uint8_t  type{0};          // MetaMsgType
    uint16_t body_size{0};     // 헤더 제외 payload 크기
    uint64_t ts{0};            // ns 등 timestamp
    uint32_t seq{0};           // 선택적 시퀀스
};
#pragma pack(pop)

#pragma pack(push, 1)
struct TrackBody { float x, y, w, h; float score; };
#pragma pack(pop)

#pragma pack(push, 1)
struct CtrlBody  { int32_t state_or_cmd; };
#pragma pack(pop)

#pragma pack(push, 1)
struct HBBody    { uint8_t reserved{0}; };
#pragma pack(pop)

// 빌더 API (MetaWire.cpp 시그니처와 일치)
MetaBuffer build_track(uint64_t ts, uint32_t seq, const cv::Rect2f& b, float score);
MetaBuffer build_aruco(uint64_t ts, int id, const cv::Rect2f& box);
MetaBuffer build_aruco_corners(uint64_t ts, int id, const std::array<cv::Point2f,4>& corners);
MetaBuffer build_aruco_full(uint64_t ts, int id, const cv::Rect2f& box,
                            const std::array<cv::Point2f,4>& corners);
MetaBuffer build_ctrl(uint64_t ts, int state_or_cmd);
MetaBuffer build_hb(uint64_t ts);

} // namespace flir

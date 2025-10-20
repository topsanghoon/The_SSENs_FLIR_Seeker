#pragma once
#include <cstdint>
#include <vector>
#include <cstring>
#include <opencv2/core.hpp>

namespace flir {

// 전송 타입(수신측에서 스위칭 용이)
enum class MetaMsgType : uint8_t { Track=1, Aruco=2, Ctrl=3, Heartbeat=4 };
constexpr uint8_t META_WIRE_VERSION = 1;

// 모든 메시지에 붙는 공통 헤더(패킹)
#pragma pack(push, 1)
struct MetaMsgHeader {
    uint8_t  version{META_WIRE_VERSION};
    uint8_t  type{0};           // MetaMsgType
    uint16_t reserved{0};
    uint32_t body_size{0};      // 뒤에 오는 body 바이트 길이
    uint64_t ts{0};             // ns (보통 본문에도 둘 필요 없지만 중복을 허용)
    uint32_t seq{0};            // frame_seq 등 (없는 타입은 0)
};
#pragma pack(pop)

// 바디들(패킹). float는 IEEE754 전제(동일 플랫폼 간 통신이면 OK)
#pragma pack(push, 1)
struct TrackBody { float x,y,w,h; float score; };
struct ArucoBody { int32_t id; float x,y,w,h; };
struct CtrlBody  { int32_t state_or_cmd; };
struct HBBody    { /* 필요시 요약 값 추가 */ };
#pragma pack(pop)

struct MetaBuffer { std::vector<uint8_t> bytes; };

// 빌더: 헤더+바디를 직렬화해 MetaBuffer로 반환
MetaBuffer build_track(uint64_t ts, uint32_t seq, const cv::Rect2f& b, float score);
MetaBuffer build_aruco(uint64_t ts, int id, const cv::Rect2f& b);
MetaBuffer build_ctrl (uint64_t ts, int state_or_cmd);
MetaBuffer build_hb   (uint64_t ts);

} // namespace flir

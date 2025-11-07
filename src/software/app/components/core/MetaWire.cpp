#include "components/includes/MetaWire.hpp"

namespace flir {

static inline void put8 (std::vector<uint8_t>& b, uint8_t v){ b.push_back(v); }
static inline void put32(std::vector<uint8_t>& b, uint32_t v){ for(int i=0;i<4;i++) b.push_back((v>>(8*i))&0xFF); }
static inline void put64(std::vector<uint8_t>& b, uint64_t v){ for(int i=0;i<8;i++) b.push_back((v>>(8*i))&0xFF); }
static inline void putf (std::vector<uint8_t>& b, float f)   {
    static_assert(sizeof(float)==4,""); uint32_t u; std::memcpy(&u,&f,4); put32(b,u);
}

enum : uint8_t { VER=1, T_TRACK=0x01, T_ARUCO=0x02, T_CTRL=0x03, T_HB=0x04 };

// flags for ARUCO
enum : uint32_t { ARU_FLAG_BBOX=1u<<0, ARU_FLAG_CORNERS=1u<<1 };

static inline void put_bytes(std::vector<uint8_t>& v, const void* p, size_t n) {
    const auto* b = static_cast<const uint8_t*>(p);
    v.insert(v.end(), b, b + n);
}

static inline MetaMsgHeader make_hdr(MetaMsgType ty, uint64_t ts, uint32_t seq, uint32_t body_size) {
    MetaMsgHeader h{};
    h.version = META_WIRE_VERSION;
    h.type    = static_cast<uint8_t>(ty);
    h.body_size = body_size;
    h.ts = ts;
    h.seq = seq;
    return h;
}

MetaBuffer build_track(uint64_t ts, uint32_t seq, const cv::Rect2f& b, float score) {
    TrackBody body{};
    body.x = b.x; body.y = b.y; body.w = b.width; body.h = b.height;
    body.score = score;

    MetaBuffer mb;
    mb.bytes.reserve(sizeof(MetaMsgHeader) + sizeof(TrackBody));
    auto hdr = make_hdr(MetaMsgType::Track, ts, seq, sizeof(TrackBody));
    put_bytes(mb.bytes, &hdr,  sizeof(hdr));
    put_bytes(mb.bytes, &body, sizeof(body));
    return mb;
}

MetaBuffer build_aruco(uint64_t ts, int id, const cv::Rect2f& box) {
    MetaBuffer w;
    put8(w.bytes, VER);
    put8(w.bytes, T_ARUCO);
    put32(w.bytes, 0);                 // reserved(align)
    put64(w.bytes, ts);
    put32(w.bytes, (uint32_t)id);
    put32(w.bytes, ARU_FLAG_BBOX);     // flags: bbox only
    putf(w.bytes, box.x); putf(w.bytes, box.y);
    putf(w.bytes, box.width); putf(w.bytes, box.height);
    return w;
}

MetaBuffer build_aruco_corners(uint64_t ts, int id, const std::array<cv::Point2f,4>& c) {
    MetaBuffer w;
    put8(w.bytes, VER);
    put8(w.bytes, T_ARUCO);
    put32(w.bytes, 0);
    put64(w.bytes, ts);
    put32(w.bytes, (uint32_t)id);
    put32(w.bytes, ARU_FLAG_CORNERS);  // flags: corners only
    for (int i=0;i<4;i++){ putf(w.bytes, c[i].x); putf(w.bytes, c[i].y); }
    return w;
}

MetaBuffer build_aruco_full(uint64_t ts, int id,
                         const cv::Rect2f& box,
                         const std::array<cv::Point2f,4>& c) {
    MetaBuffer w;
    put8(w.bytes, VER);
    put8(w.bytes, T_ARUCO);
    put32(w.bytes, 0);
    put64(w.bytes, ts);
    put32(w.bytes, (uint32_t)id);
    put32(w.bytes, ARU_FLAG_BBOX | ARU_FLAG_CORNERS); // 둘 다
    // bbox
    putf(w.bytes, box.x); putf(w.bytes, box.y);
    putf(w.bytes, box.width); putf(w.bytes, box.height);
    // corners p0..p3
    for (int i=0;i<4;i++){ putf(w.bytes, c[i].x); putf(w.bytes, c[i].y); }
    return w;
}

MetaBuffer build_ctrl(uint64_t ts, int state_or_cmd) {
    MetaBuffer w;
    put8(w.bytes, VER);            // 0x01
    put8(w.bytes, T_CTRL);         // 0x03
    put8(w.bytes, 0);              // rsv(1)
    put8(w.bytes, 0);              // rsv(1)  ← 총 헤더 4바이트 맞춤

    put64(w.bytes, ts);                                  // p+0 .. p+7
    put32(w.bytes, static_cast<uint32_t>(state_or_cmd)); // p+8 .. p+11

    return w;
}

MetaBuffer build_hb(uint64_t ts) {
    HBBody body{};
    MetaBuffer mb;
    mb.bytes.reserve(sizeof(MetaMsgHeader) + sizeof(HBBody));
    auto hdr = make_hdr(MetaMsgType::Heartbeat, ts, 0, sizeof(HBBody));
    put_bytes(mb.bytes, &hdr,  sizeof(hdr));
    put_bytes(mb.bytes, &body, sizeof(body));
    return mb;
}

} // namespace flir

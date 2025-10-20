#include "components/includes/MetaWire.hpp"

namespace flir {

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

MetaBuffer build_aruco(uint64_t ts, int id, const cv::Rect2f& b) {
    ArucoBody body{};
    body.id = id;
    body.x = b.x; body.y = b.y; body.w = b.width; body.h = b.height;

    MetaBuffer mb;
    mb.bytes.reserve(sizeof(MetaMsgHeader) + sizeof(ArucoBody));
    auto hdr = make_hdr(MetaMsgType::Aruco, ts, 0, sizeof(ArucoBody));
    put_bytes(mb.bytes, &hdr,  sizeof(hdr));
    put_bytes(mb.bytes, &body, sizeof(body));
    return mb;
}

MetaBuffer build_ctrl(uint64_t ts, int state_or_cmd) {
    CtrlBody body{};
    body.state_or_cmd = state_or_cmd;

    MetaBuffer mb;
    mb.bytes.reserve(sizeof(MetaMsgHeader) + sizeof(CtrlBody));
    auto hdr = make_hdr(MetaMsgType::Ctrl, ts, 0, sizeof(CtrlBody));
    put_bytes(mb.bytes, &hdr,  sizeof(hdr));
    put_bytes(mb.bytes, &body, sizeof(body));
    return mb;
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

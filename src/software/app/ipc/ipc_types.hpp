#pragma once
#include <cstdint>
#include <variant>
#include <opencv2/core.hpp>

namespace flir {

// ---------- Topics ----------
enum class Topic : int { Tracking=0, Aruco=1, Control=2 };

// ---------- Events ----------
enum class EventType : uint8_t { Init, Track, Lost, NeedReselect, Aruco, MetaCtrl, CtrlState };

struct InitEvent         { cv::Rect2f box; uint64_t ts; uint32_t frame_seq; };
struct TrackEvent        { cv::Rect2f box; float score; uint64_t ts; uint32_t frame_seq; };
struct LostEvent         { cv::Rect2f last_box; uint64_t ts; uint32_t frame_seq; };
struct NeedReselectEvent { };

struct ArucoEvent        { int id; cv::Rect2f box; /*pose?*/ uint64_t ts; };
struct MetaCtrlEvent     { int cmd; uint64_t ts; };
struct CtrlStateEvent    { int state; uint64_t ts; };

struct Event {
    EventType type;
    std::variant<
        InitEvent, TrackEvent, LostEvent, NeedReselectEvent,
        ArucoEvent, MetaCtrlEvent, CtrlStateEvent
    > payload;
};

// ---------- Commands ----------
enum class CmdType : uint8_t { CLICK };

struct UserCmd {
    CmdType     type;
    cv::Rect2f  box;
    uint32_t    seq;
};

} // namespace flir

// unordered_map에서 enum class Topic 사용 위한 해시
namespace std {
template <> struct hash<flir::Topic> {
    size_t operator()(const flir::Topic t) const noexcept {
        return std::hash<int>()(static_cast<int>(t));
    }
};
}

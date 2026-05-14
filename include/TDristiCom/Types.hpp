#pragma once
#include <cstdint>
#include <string>

namespace robotipc {

// ─── Wire frame ───────────────────────────────────────────────────────────────
constexpr uint16_t FRAME_MAGIC       = 0xCAFE;
constexpr size_t   FRAME_HEADER_SIZE = 6;        // 2B magic + 4B length
constexpr size_t   MAX_PAYLOAD_SIZE  = 256*1024; // 256 KB

// ─── Transport mode ───────────────────────────────────────────────────────────
enum class TransportMode { UNIX, TCP };
enum class StreamMode    { MMAP, UDP  };

// ─── Service ──────────────────────────────────────────────────────────────────
enum class ServiceStatus : uint8_t { OK=0, ERROR=1, TIMEOUT=2 };

// ─── Action goal state machine ────────────────────────────────────────────────
enum class GoalStatus : uint8_t {
    IDLE=0, ACCEPTED=1, REJECTED=2, EXECUTING=3,
    CANCELING=4, SUCCEEDED=5, CANCELED=6, ABORTED=7
};
inline const char* goalStatusStr(GoalStatus s){
    switch(s){
        case GoalStatus::IDLE:      return "IDLE";
        case GoalStatus::ACCEPTED:  return "ACCEPTED";
        case GoalStatus::REJECTED:  return "REJECTED";
        case GoalStatus::EXECUTING: return "EXECUTING";
        case GoalStatus::CANCELING: return "CANCELING";
        case GoalStatus::SUCCEEDED: return "SUCCEEDED";
        case GoalStatus::CANCELED:  return "CANCELED";
        case GoalStatus::ABORTED:   return "ABORTED";
        default:                    return "UNKNOWN";
    }
}

// ─── Action wire msg types ────────────────────────────────────────────────────
enum class ActionMsgType : uint8_t {
    GOAL_REQUEST=0x01, GOAL_ACK=0x02,
    CANCEL_REQUEST=0x03, CANCEL_ACK=0x04,
    RESULT_REQUEST=0x05, RESULT_RESPONSE=0x06,
    FEEDBACK=0x07
};

// ─── Goal ID ──────────────────────────────────────────────────────────────────
struct GoalID {
    uint64_t id{0};
    bool operator==(const GoalID& o) const { return id==o.id; }
    std::string str() const { return "goal-"+std::to_string(id); }
};

// ─── Session ──────────────────────────────────────────────────────────────────
enum class SessionState : uint8_t {
    DISCONNECTED=0, CONNECTING=1, CONNECTED=2, RECONNECTING=3
};

// ─── Stream packet header (UDP) ───────────────────────────────────────────────
#pragma pack(push,1)
struct StreamHeader {
    uint16_t magic{0xDA7A};   // "DATA"
    uint16_t topic_id;        // which stream
    uint32_t seq;             // sequence number (detect drops/reorder)
    uint64_t timestamp_us;    // microseconds since epoch
    uint32_t payload_len;
};
#pragma pack(pop)
constexpr uint16_t STREAM_MAGIC = 0xDA7A;

} // namespace robotipc

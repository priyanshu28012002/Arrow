#pragma once
#include <functional>
#include <cstdint>

namespace robotipc {

// ─────────────────────────────────────────────────────────────────────────────
//  IStream<T>  —  Abstract real-time, lossy, newest-wins stream
//  Implementations: MmapStream (same host), UdpStream (LAN)
// ─────────────────────────────────────────────────────────────────────────────
template<typename T>
class IStream {
public:
    virtual ~IStream() = default;

    // Publisher side
    virtual void publish(const T& msg)                             = 0;

    // Subscriber side — poll newest value (non-blocking)
    virtual bool latest(T& out)                                    = 0;

    // Subscriber side — callback mode (background thread)
    virtual void subscribe(std::function<void(const T&)> cb)       = 0;

    // Stop background subscriber thread
    virtual void unsubscribe()                                     = 0;
};

} // namespace robotipc

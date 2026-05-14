#pragma once
#include <vector>
#include <cstdint>
#include <string>
#include <memory>

namespace robotipc {

// ─────────────────────────────────────────────────────────────────────────────
//  ITransport  —  Abstract reliable, ordered, framed transport
//  Implementations: UnixTransport (same host), TcpTransport (LAN)
// ─────────────────────────────────────────────────────────────────────────────
class ITransport {
public:
    virtual ~ITransport() = default;

    virtual void sendFrame(const std::vector<uint8_t>& frame)    = 0;
    virtual std::vector<uint8_t> recvFrame(int timeout_ms = -1)  = 0;
    virtual bool   isConnected() const                           = 0;
    virtual void   close()                                       = 0;
    virtual int    fd() const { return -1; }  // optional: raw fd for poll()
};

// ─────────────────────────────────────────────────────────────────────────────
//  ITransportServer  —  Listens and produces ITransport per accepted client
// ─────────────────────────────────────────────────────────────────────────────
class ITransportServer {
public:
    virtual ~ITransportServer() = default;

    virtual void listen()                                        = 0;
    virtual std::unique_ptr<ITransport> accept()                 = 0;
    virtual void close()                                         = 0;
};

} // namespace robotipc

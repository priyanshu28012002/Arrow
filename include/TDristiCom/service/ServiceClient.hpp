#pragma once
#include "../transport/UnixTransport.hpp"
#include "../transport/TcpTransport.hpp"
#include "../Serializer.hpp"
#include "../Types.hpp"
#include "../Logger.hpp"

#include <string>
#include <chrono>
#include <stdexcept>

namespace robotipc {

template<typename Req, typename Res>
class ServiceClient {
public:
    // Unix client
    explicit ServiceClient(const std::string& path, int timeout_ms=5000)
        : mode_(Mode::UNIX), path_(path), timeout_ms_(timeout_ms) {}

    // TCP client
    ServiceClient(const std::string& host, uint16_t port, int timeout_ms=5000)
        : mode_(Mode::TCP), host_(host), port_(port), timeout_ms_(timeout_ms) {}

    Res call(const Req& req, int timeout_ms=-1){
        if(timeout_ms<0) timeout_ms=timeout_ms_;
        if(mode_==Mode::UNIX){
            UnixTransport t(path_, timeout_ms);
            t.sendFrame(Serializer::encodeStruct(req));
            return Serializer::decodeStruct<Res>(t.recvFrame(timeout_ms));
        } else {
            TcpTransport t(host_, port_, timeout_ms);
            t.sendFrame(Serializer::encodeStruct(req));
            return Serializer::decodeStruct<Res>(t.recvFrame(timeout_ms));
        }
    }

    ServiceStatus tryCall(const Req& req, Res& out, int timeout_ms=-1){
        try{ out=call(req,timeout_ms); return ServiceStatus::OK; }
        catch(const std::runtime_error& e){
            std::string m=e.what();
            if(m.find("timeout")!=std::string::npos) return ServiceStatus::TIMEOUT;
            RIPC_ERROR("ServiceClient",m);
            return ServiceStatus::ERROR;
        }
    }

private:
    enum class Mode { UNIX, TCP };
    Mode        mode_;
    std::string path_, host_;
    uint16_t    port_{0};
    int         timeout_ms_;
};

} // namespace robotipc

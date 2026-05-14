#pragma once
#include "../transport/ITransport.hpp"
#include "../transport/UnixTransport.hpp"
#include "../transport/TcpTransport.hpp"
#include "../Serializer.hpp"
#include "../Logger.hpp"

#include <functional>
#include <thread>
#include <atomic>
#include <memory>
#include <string>

namespace robotipc {

// ─────────────────────────────────────────────────────────────────────────────
//  ServiceServer<Req, Res>
//  Transport-agnostic: pass any ITransportServer (Unix or TCP)
//
//  Quick factory helpers:
//    ServiceServer<Req,Res>::makeUnix("/tmp/svc")
//    ServiceServer<Req,Res>::makeTcp(7401)
// ─────────────────────────────────────────────────────────────────────────────
template<typename Req, typename Res>
class ServiceServer {
public:
    using Handler = std::function<Res(const Req&)>;

    explicit ServiceServer(std::unique_ptr<ITransportServer> srv)
        : srv_(std::move(srv)), running_(false) {}

    // Factory helpers
    static std::unique_ptr<ServiceServer<Req,Res>> makeUnix(const std::string& path){
        return std::make_unique<ServiceServer<Req,Res>>(
            std::make_unique<UnixTransportServer>(path));
    }
    static std::unique_ptr<ServiceServer<Req,Res>> makeTcp(uint16_t port){
        return std::make_unique<ServiceServer<Req,Res>>(
            std::make_unique<TcpTransportServer>(port));
    }

    ~ServiceServer(){ shutdown(); }

    void bind(Handler h){ handler_=std::move(h); }

    void spin(){
        if(!handler_) throw std::runtime_error("No handler bound");
        srv_->listen();
        running_.store(true);
        RIPC_INFO("ServiceServer","Ready");
        while(running_.load()){
            try{
                auto client = srv_->accept();
                if(!running_.load()) break;
                std::thread([this, c=std::move(client)]() mutable {
                    handle(std::move(c));
                }).detach();
            } catch(const std::exception& e){
                if(running_.load()) RIPC_ERROR("ServiceServer",e.what());
            }
        }
        srv_->close();
    }

    void spinAsync(){
        spin_thread_ = std::thread([this]{ spin(); });
    }

    void shutdown(){
        running_.store(false);
        srv_->close();
        if(spin_thread_.joinable()) spin_thread_.join();
    }

private:
    void handle(std::unique_ptr<ITransport> client){
        try{
            auto payload = client->recvFrame(5000);
            Req  req     = Serializer::decodeStruct<Req>(payload);
            Res  res     = handler_(req);
            client->sendFrame(Serializer::encodeStruct(res));
            // Graceful drain
            uint8_t drain[64];
            while(::read(client->fd(), drain, sizeof(drain))>0){}
        } catch(const std::exception& e){
            RIPC_ERROR("ServiceServer::handle",e.what());
        }
    }

    std::unique_ptr<ITransportServer> srv_;
    Handler                           handler_;
    std::atomic<bool>                 running_;
    std::thread                       spin_thread_;
};

} // namespace robotipc

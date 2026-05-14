#pragma once
#include "../transport/TcpTransport.hpp"
#include "../Types.hpp"
#include "../Logger.hpp"

#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <string>

namespace robotipc {

// ─────────────────────────────────────────────────────────────────────────────
//  SessionManager  —  Persistent TCP session with heartbeat + reconnect
//
//  Control PC side (client):
//    SessionManager session("192.168.1.50", 7400);
//    session.onConnected([](){ ... });
//    session.onDisconnected([](){ stop_robot(); });
//    session.connect();          // starts reconnect loop
//
//  Robot side (server):
//    SessionManager session(7400);
//    session.onConnected([](){ ... });
//    session.start();            // starts accept loop
// ─────────────────────────────────────────────────────────────────────────────

// Heartbeat wire message
#pragma pack(push,1)
struct HeartbeatMsg {
    uint32_t magic{45};
    uint64_t session_id;         // random ID assigned at connect time
    uint64_t timestamp_us;       // sender's clock
    uint8_t  type;               // 0=PING, 1=PONG
};
#pragma pack(pop)

class SessionManager {
public:
    using Callback = std::function<void()>;

    // ── Client (Control PC) constructor ──────────────────────────────────────
    SessionManager(const std::string& host, uint16_t port,
                   int heartbeat_ms=100, int reconnect_ms=1000)
        : mode_(Mode::CLIENT), host_(host), port_(port)
        , heartbeat_ms_(heartbeat_ms), reconnect_ms_(reconnect_ms)
        , state_(SessionState::DISCONNECTED), running_(false)
    {}

    // ── Server (Robot) constructor ────────────────────────────────────────────
    explicit SessionManager(uint16_t port,
                            int heartbeat_ms=100)
        : mode_(Mode::SERVER), port_(port)
        , heartbeat_ms_(heartbeat_ms), reconnect_ms_(0)
        , state_(SessionState::DISCONNECTED), running_(false)
    {}

    ~SessionManager(){ stop(); }

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void onConnected   (Callback cb){ on_connected_    = std::move(cb); }
    void onDisconnected(Callback cb){ on_disconnected_ = std::move(cb); }
    void onReconnecting(Callback cb){ on_reconnecting_ = std::move(cb); }

    // ── Start ─────────────────────────────────────────────────────────────────
    void start(){
        running_.store(true);
        if (mode_==Mode::CLIENT)
            main_thread_ = std::thread([this]{ clientLoop(); });
        else
            main_thread_ = std::thread([this]{ serverLoop(); });
    }

    void stop(){
        running_.store(false);
        {
            std::lock_guard<std::mutex> lk(sock_mutex_);
            if (ctrl_sock_) { ctrl_sock_->close(); ctrl_sock_.reset(); }
        }
        if (main_thread_.joinable()) main_thread_.join();
        if (hb_thread_.joinable())   hb_thread_.join();
    }

    SessionState state() const { return state_.load(); }
    bool isConnected()   const { return state()==SessionState::CONNECTED; }

    // Block until connected (or timeout_ms)
    bool waitConnected(int timeout_ms=5000){
        auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
        while(std::chrono::steady_clock::now() < deadline){
            if(isConnected()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        return false;
    }

private:
    enum class Mode { CLIENT, SERVER };

    // ── Client reconnect loop ─────────────────────────────────────────────────
    void clientLoop(){
        while(running_.load()){
            state_.store(SessionState::CONNECTING);
            RIPC_INFO("Session","Connecting to "+host_+":"+std::to_string(port_));
            try {
                auto sock = std::make_unique<TcpTransport>(host_, port_, 3000);
                {
                    std::lock_guard<std::mutex> lk(sock_mutex_);
                    ctrl_sock_ = std::move(sock);
                }
                state_.store(SessionState::CONNECTED);
                RIPC_INFO("Session","Connected to "+host_+":"+std::to_string(port_));
                if(on_connected_) on_connected_();
                startHeartbeat();
                heartbeatClientLoop();   // blocks until disconnect
            } catch(const std::exception& e){
                RIPC_WARN("Session",std::string("Connect failed: ")+e.what());
            }
            if(!running_.load()) break;
            state_.store(SessionState::RECONNECTING);
            RIPC_WARN("Session","Reconnecting in "+std::to_string(reconnect_ms_)+"ms...");
            if(on_disconnected_) on_disconnected_();
            if(on_reconnecting_) on_reconnecting_();
            std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_ms_));
        }
        state_.store(SessionState::DISCONNECTED);
    }

    // ── Server accept loop ────────────────────────────────────────────────────
    void serverLoop(){
        TcpTransportServer srv(port_);
        srv.listen();
        RIPC_INFO("Session","Session server ready on port "+std::to_string(port_));

        while(running_.load()){
            try {
                auto sock = srv.accept();
                {
                    std::lock_guard<std::mutex> lk(sock_mutex_);
                    ctrl_sock_ = std::move(sock);
                }
                state_.store(SessionState::CONNECTED);
                RIPC_INFO("Session","Control PC connected");
                if(on_connected_) on_connected_();
                startHeartbeat();
                heartbeatServerLoop();   // blocks until disconnect
                state_.store(SessionState::DISCONNECTED);
                if(on_disconnected_) on_disconnected_();
            } catch(const std::exception& e){
                if(running_.load())
                    RIPC_WARN("Session",std::string("Session error: ")+e.what());
            }
        }
    }

    // ── Heartbeat ─────────────────────────────────────────────────────────────
    void startHeartbeat(){
        if(hb_thread_.joinable()) hb_thread_.join();
    }

    void heartbeatClientLoop(){
        // Send PING every heartbeat_ms_, expect PONG within 3x
        int missed = 0;
        while(running_.load() && isConnected()){
            std::this_thread::sleep_for(std::chrono::milliseconds(heartbeat_ms_));
            try {
                HeartbeatMsg ping{};
                ping.session_id   = session_id_;
                ping.timestamp_us = nowUs();
                ping.type         = 0;  // PING
                std::lock_guard<std::mutex> lk(sock_mutex_);
                if(!ctrl_sock_) break;
                ctrl_sock_->sendFrame(Serializer::encodeStruct(ping));

                // Wait for PONG (brief timeout)
                auto pong = ctrl_sock_->recvFrame(heartbeat_ms_*3);
                auto msg  = Serializer::decodeStruct<HeartbeatMsg>(pong);
                if(msg.type==1) missed=0;
            } catch(...){
                if(++missed >= 3){
                    RIPC_WARN("Session","Heartbeat lost — disconnecting");
                    std::lock_guard<std::mutex> lk(sock_mutex_);
                    if(ctrl_sock_) ctrl_sock_->close();
                    break;
                }
            }
        }
    }

    void heartbeatServerLoop(){
        // Receive PING, send PONG
        while(running_.load() && isConnected()){
            try {
                std::unique_lock<std::mutex> lk(sock_mutex_);
                if(!ctrl_sock_) break;
                auto payload = ctrl_sock_->recvFrame(heartbeat_ms_*5);
                lk.unlock();

                auto msg = Serializer::decodeStruct<HeartbeatMsg>(payload);
                if(msg.type==0){ // PING → PONG
                    HeartbeatMsg pong = msg;
                    pong.type = 1;
                    pong.timestamp_us = nowUs();
                    std::lock_guard<std::mutex> lk2(sock_mutex_);
                    if(ctrl_sock_) ctrl_sock_->sendFrame(Serializer::encodeStruct(pong));
                }
            } catch(const std::exception& e){
                RIPC_WARN("Session",std::string("Heartbeat server: ")+e.what());
                break;
            }
        }
    }

    static uint64_t nowUs(){
        return uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    Mode     mode_;
    std::string host_;
    uint16_t port_;
    int      heartbeat_ms_, reconnect_ms_;

    std::atomic<SessionState> state_;
    std::atomic<bool>         running_;
    uint64_t                  session_id_{0xDEADBEEF12345678ULL};

    std::unique_ptr<ITransport> ctrl_sock_;
    std::mutex                  sock_mutex_;

    std::thread main_thread_, hb_thread_;

    Callback on_connected_, on_disconnected_, on_reconnecting_;
};

} // namespace robotipc

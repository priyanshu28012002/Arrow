#pragma once
#include "IStream.hpp"
#include "../Types.hpp"
#include "../Logger.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <cstring>
#include <cerrno>
#include <atomic>
#include <thread>
#include <functional>
#include <stdexcept>
#include <chrono>

namespace robotipc {

// ─────────────────────────────────────────────────────────────────────────────
//  UdpStream<T>  —  IStream over UDP (LAN / robot over Ethernet)
//  Best for: joystick → robot, lidar/IMU/odom → control PC
//
//  Properties:
//   • Fire-and-forget sendto() — no blocking, no backpressure
//   • Sequence numbers — out-of-order packets silently dropped
//   • Timestamp in every packet — latency measurement possible
//   • Newest-wins — subscriber always sees most recent value
// ─────────────────────────────────────────────────────────────────────────────
template<typename T>
class UdpStream : public IStream<T> {
public:
    // Publisher: binds nothing, sends to dst_ip:dst_port
    static std::unique_ptr<UdpStream<T>> makePublisher(
            const std::string& dst_ip, uint16_t dst_port, uint16_t topic_id=0)
    {
        auto s = std::unique_ptr<UdpStream<T>>(new UdpStream<T>());
        s->topic_id_ = topic_id;
        s->fd_ = ::socket(AF_INET,SOCK_DGRAM,0);
        if(s->fd_<0) throw std::runtime_error("udp socket: "+se());

        s->dst_.sin_family      = AF_INET;
        s->dst_.sin_port        = htons(dst_port);
        ::inet_pton(AF_INET, dst_ip.c_str(), &s->dst_.sin_addr);

        // Allow broadcast
        int br=1; ::setsockopt(s->fd_,SOL_SOCKET,SO_BROADCAST,&br,sizeof(br));
        RIPC_DEBUG("UdpStream","Publisher -> "+dst_ip+":"+std::to_string(dst_port));
        return s;
    }

    // Subscriber: binds to local port, receives from anyone
    static std::unique_ptr<UdpStream<T>> makeSubscriber(
            uint16_t bind_port, uint16_t topic_id=0)
    {
        auto s = std::unique_ptr<UdpStream<T>>(new UdpStream<T>());
        s->topic_id_ = topic_id;
        s->fd_ = ::socket(AF_INET,SOCK_DGRAM,0);
        if(s->fd_<0) throw std::runtime_error("udp socket: "+se());

        int on=1;
        ::setsockopt(s->fd_,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on));
        ::setsockopt(s->fd_,SOL_SOCKET,SO_REUSEPORT,&on,sizeof(on));

        sockaddr_in a{};
        a.sin_family=AF_INET; a.sin_port=htons(bind_port);
        a.sin_addr.s_addr=INADDR_ANY;
        if(::bind(s->fd_,(sockaddr*)&a,sizeof(a))<0)
            throw std::runtime_error("udp bind(:"+std::to_string(bind_port)+"): "+se());

        RIPC_DEBUG("UdpStream","Subscriber on port "+std::to_string(bind_port));
        return s;
    }

    ~UdpStream() override {
        unsubscribe();
        if(fd_>=0){ ::close(fd_); fd_=-1; }
    }

    // ── Publisher ─────────────────────────────────────────────────────────────
    void publish(const T& msg) override {
        Packet pkt{};
        pkt.hdr.magic       = STREAM_MAGIC;
        pkt.hdr.topic_id    = topic_id_;
        pkt.hdr.seq         = seq_.fetch_add(1, std::memory_order_relaxed);
        pkt.hdr.timestamp_us= nowUs();
        pkt.hdr.payload_len = sizeof(T);
        pkt.payload         = msg;

        ::sendto(fd_, &pkt, sizeof(pkt), 0,
                 (sockaddr*)&dst_, sizeof(dst_));
        // Intentionally ignore send errors — UDP is fire-and-forget
    }

    // ── Subscriber ────────────────────────────────────────────────────────────
    bool latest(T& out) override {
        if (!has_data_.load()) return false;
        out = latest_;
        return true;
    }

    void subscribe(std::function<void(const T&)> cb) override {
        cb_ = std::move(cb);
        running_.store(true);
        sub_thread_ = std::thread([this]{ recvLoop(); });
    }

    void unsubscribe() override {
        running_.store(false);
        if(sub_thread_.joinable()) sub_thread_.join();
    }

    // Stats
    uint64_t droppedPackets() const { return dropped_.load(); }
    uint64_t receivedPackets()const { return received_.load(); }

private:
    // On-wire packet: header + typed payload (fixed size, no fragmentation)
    struct Packet {
        StreamHeader hdr;
        T            payload;
    };
    static_assert(sizeof(Packet) < 65507, "UDP payload exceeds max datagram size");

    UdpStream() = default;

    void recvLoop(){
        Packet pkt;
        uint32_t expected_seq = 0;
        bool     first        = true;

        while(running_.load()){
            pollfd p{fd_,POLLIN,0};
            if(::poll(&p,1,50)<=0) continue;

            sockaddr_in src{}; socklen_t sl=sizeof(src);
            ssize_t n = ::recvfrom(fd_,&pkt,sizeof(pkt),0,(sockaddr*)&src,&sl);
            if(n<=0) continue;
            if(size_t(n) < sizeof(Packet)) continue;
            if(pkt.hdr.magic != STREAM_MAGIC) continue;

            received_.fetch_add(1);

            // Drop out-of-order / stale packets
            if(!first && pkt.hdr.seq < expected_seq){
                dropped_.fetch_add(1);
                continue;
            }
            if(!first && pkt.hdr.seq > expected_seq+100)
                RIPC_WARN("UdpStream","Seq jump: expected "+
                          std::to_string(expected_seq)+" got "+
                          std::to_string(pkt.hdr.seq));

            first        = false;
            expected_seq = pkt.hdr.seq + 1;
            latest_      = pkt.payload;
            has_data_.store(true);

            if(cb_) cb_(pkt.payload);
        }
    }

    static uint64_t nowUs(){
        return uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    int              fd_{-1};
    uint16_t         topic_id_{0};
    sockaddr_in      dst_{};
    std::atomic<uint32_t> seq_{0};

    T                     latest_{};
    std::atomic<bool>     has_data_{false};
    std::function<void(const T&)> cb_;
    std::atomic<bool>     running_{false};
    std::thread           sub_thread_;
    std::atomic<uint64_t> dropped_{0}, received_{0};

    static std::string se(){ return std::string(strerror(errno)); }
};

} // namespace robotipc

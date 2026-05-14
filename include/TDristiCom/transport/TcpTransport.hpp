#pragma once
#include "ITransport.hpp"
#include "../Serializer.hpp"
#include "../Logger.hpp"

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cstring>
#include <cerrno>
#include <stdexcept>
#include <string>

namespace robotipc {

// ─────────────────────────────────────────────────────────────────────────────
//  TcpTransport  —  ITransport over TCP (LAN / robot over Ethernet)
//  Best for: Control PC <-> Robot over LAN cable, reliable control messages
// ─────────────────────────────────────────────────────────────────────────────
class TcpTransport : public ITransport {
public:
    // From accepted fd (server side)
    explicit TcpTransport(int fd) : fd_(fd) { applyKeepAlive(); }

    // Client: connect to host:port
    TcpTransport(const std::string& host, uint16_t port, int timeout_ms=3000)
        : fd_(-1)
    {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_<0) throw std::runtime_error("socket: "+se());

        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port   = htons(port);
        if (::inet_pton(AF_INET, host.c_str(), &a.sin_addr)<=0)
            throw std::runtime_error("Bad IP: "+host);

        setNonBlock(true);
        int rc = ::connect(fd_,(sockaddr*)&a,sizeof(a));
        if (rc<0 && errno!=EINPROGRESS)
            throw std::runtime_error("connect("+host+":"+std::to_string(port)+"): "+se());
        if (rc!=0) {
            pollfd p{fd_,POLLOUT,0};
            if (::poll(&p,1,timeout_ms)<=0)
                throw std::runtime_error("connect timeout "+host+":"+std::to_string(port));
            int err=0; socklen_t el=sizeof(err);
            ::getsockopt(fd_,SOL_SOCKET,SO_ERROR,&err,&el);
            if (err) throw std::runtime_error("connect err: "+std::string(strerror(err)));
        }
        setNonBlock(false);
        applyKeepAlive();
        setNoDelay(true);
    }

    ~TcpTransport() override { close(); }
    TcpTransport(TcpTransport&& o) noexcept : fd_(o.fd_){ o.fd_=-1; }
    TcpTransport(const TcpTransport&) = delete;

    void sendFrame(const std::vector<uint8_t>& frame) override {
        writeAll(frame.data(), frame.size());
    }

    template<typename T>
    void sendStruct(const T& obj){ sendFrame(Serializer::encodeStruct(obj)); }

    std::vector<uint8_t> recvFrame(int timeout_ms=-1) override {
        uint8_t hdr[FRAME_HEADER_SIZE];
        readAll(hdr, FRAME_HEADER_SIZE, timeout_ms);
        int32_t pl = Serializer::peekLength(hdr);
        if (pl<0) throw std::runtime_error("Bad frame magic");
        std::vector<uint8_t> full(FRAME_HEADER_SIZE+pl);
        std::memcpy(full.data(), hdr, FRAME_HEADER_SIZE);
        readAll(full.data()+FRAME_HEADER_SIZE, pl, timeout_ms);
        return Serializer::decode(full.data(), full.size());
    }

    template<typename T>
    T recvStruct(int tms=-1){ return Serializer::decodeStruct<T>(recvFrame(tms)); }

    bool isConnected() const override { return fd_>=0; }
    int  fd()          const override { return fd_; }

    void close() override {
        if(fd_>=0){ ::shutdown(fd_,SHUT_RDWR); ::close(fd_); fd_=-1; }
    }

private:
    int fd_;

    void applyKeepAlive(){
        int on=1;
        ::setsockopt(fd_,SOL_SOCKET,SO_KEEPALIVE,&on,sizeof(on));
        int idle=5, intvl=1, cnt=3;
        ::setsockopt(fd_,IPPROTO_TCP,TCP_KEEPIDLE, &idle, sizeof(idle));
        ::setsockopt(fd_,IPPROTO_TCP,TCP_KEEPINTVL,&intvl,sizeof(intvl));
        ::setsockopt(fd_,IPPROTO_TCP,TCP_KEEPCNT,  &cnt,  sizeof(cnt));
    }

    void setNoDelay(bool on){
        int v=on?1:0;
        ::setsockopt(fd_,IPPROTO_TCP,TCP_NODELAY,&v,sizeof(v));
    }

    void setNonBlock(bool on){
        int f=::fcntl(fd_,F_GETFL,0);
        ::fcntl(fd_,F_SETFL, on ? f|O_NONBLOCK : f&~O_NONBLOCK);
    }

    void writeAll(const uint8_t* d, size_t n){
        size_t s=0;
        while(s<n){
            ssize_t r=::send(fd_,d+s,n-s,MSG_NOSIGNAL);
            if(r<=0) throw std::runtime_error("send: "+se());
            s+=r;
        }
    }

    void readAll(uint8_t* buf, size_t n, int tms){
        size_t g=0;
        while(g<n){
            if(tms>=0){
                pollfd p{fd_,POLLIN,0};
                int r=::poll(&p,1,tms);
                if(r==0) throw std::runtime_error("recv timeout");
                if(r<0)  throw std::runtime_error("poll: "+se());
                if((p.revents&POLLERR)&&!(p.revents&POLLIN))
                    throw std::runtime_error("socket error");
                if((p.revents&POLLHUP)&&!(p.revents&POLLIN))
                    throw std::runtime_error("peer closed");
            }
            ssize_t r=::recv(fd_,buf+g,n-g,0);
            if(r==0) throw std::runtime_error("connection closed");
            if(r<0){ if(errno==EINTR) continue; throw std::runtime_error("recv: "+se()); }
            g+=r;
        }
    }

    static std::string se(){ return std::string(strerror(errno)); }
};

// ─────────────────────────────────────────────────────────────────────────────
//  TcpTransportServer
// ─────────────────────────────────────────────────────────────────────────────
class TcpTransportServer : public ITransportServer {
public:
    TcpTransportServer(uint16_t port, int backlog=16)
        : port_(port), fd_(-1), backlog_(backlog) {}

    ~TcpTransportServer() override { close(); }

    void listen() override {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_<0) throw std::runtime_error("socket: "+se());

        int on=1;
        ::setsockopt(fd_,SOL_SOCKET,SO_REUSEADDR,&on,sizeof(on));
        ::setsockopt(fd_,SOL_SOCKET,SO_REUSEPORT,&on,sizeof(on));

        sockaddr_in a{};
        a.sin_family      = AF_INET;
        a.sin_port        = htons(port_);
        a.sin_addr.s_addr = INADDR_ANY;

        if (::bind(fd_,(sockaddr*)&a,sizeof(a))<0)
            throw std::runtime_error("bind(:"+std::to_string(port_)+"): "+se());
        if (::listen(fd_,backlog_)<0)
            throw std::runtime_error("listen: "+se());

        RIPC_DEBUG("TcpServer","Listening on port "+std::to_string(port_));
    }

    std::unique_ptr<ITransport> accept() override {
        sockaddr_in peer{}; socklen_t len=sizeof(peer);
        int cfd = ::accept(fd_,(sockaddr*)&peer,&len);
        if (cfd<0) throw std::runtime_error("accept: "+se());

        char ip[INET_ADDRSTRLEN];
        ::inet_ntop(AF_INET,&peer.sin_addr,ip,sizeof(ip));
        RIPC_DEBUG("TcpServer","Client connected: "+std::string(ip)+
                   ":"+std::to_string(ntohs(peer.sin_port)));
        return std::make_unique<TcpTransport>(cfd);
    }

    void close() override {
        if(fd_>=0){ ::close(fd_); fd_=-1; }
    }

    int fd() const { return fd_; }

private:
    uint16_t port_; int fd_, backlog_;
    static std::string se(){ return std::string(strerror(errno)); }
};

} // namespace robotipc

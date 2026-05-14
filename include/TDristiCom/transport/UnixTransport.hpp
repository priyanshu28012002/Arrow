#pragma once
#include "ITransport.hpp"
#include "../Serializer.hpp"
#include "../Logger.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cstring>
#include <cerrno>
#include <stdexcept>
#include <string>

namespace robotipc {

// ─────────────────────────────────────────────────────────────────────────────
//  UnixTransport  —  ITransport over Unix domain socket (same host only)
//  Best for: simulation, local nodes, unit tests, intra-process IPC
// ─────────────────────────────────────────────────────────────────────────────
class UnixTransport : public ITransport {
public:
    // Construct from already-accepted fd (server side)
    explicit UnixTransport(int fd) : fd_(fd) {}

    // Construct as client and connect immediately
    UnixTransport(const std::string& path, int timeout_ms=3000)
        : fd_(-1)
    {
        fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd_<0) throw std::runtime_error("socket: "+se());

        setNonBlock(true);
        sockaddr_un a{}; a.sun_family=AF_UNIX;
        std::strncpy(a.sun_path, path.c_str(), sizeof(a.sun_path)-1);
        int rc = ::connect(fd_, (sockaddr*)&a, sizeof(a));
        if (rc<0 && errno!=EINPROGRESS) throw std::runtime_error("connect("+path+"): "+se());
        if (rc!=0) {
            pollfd p{fd_, POLLOUT, 0};
            if (::poll(&p,1,timeout_ms)<=0) throw std::runtime_error("connect timeout: "+path);
            int err=0; socklen_t el=sizeof(err);
            ::getsockopt(fd_, SOL_SOCKET, SO_ERROR, &err, &el);
            if (err) throw std::runtime_error("connect failed: "+std::string(strerror(err)));
        }
        setNonBlock(false);
    }

    ~UnixTransport() override { close(); }

    UnixTransport(UnixTransport&& o) noexcept : fd_(o.fd_) { o.fd_=-1; }
    UnixTransport(const UnixTransport&) = delete;

    void sendFrame(const std::vector<uint8_t>& frame) override {
        writeAll(frame.data(), frame.size());
    }

    template<typename T>
    void sendStruct(const T& obj) { sendFrame(Serializer::encodeStruct(obj)); }

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
    T recvStruct(int timeout_ms=-1){
        return Serializer::decodeStruct<T>(recvFrame(timeout_ms));
    }

    bool isConnected() const override { return fd_>=0; }
    int  fd()          const override { return fd_; }

    void close() override {
        if (fd_>=0){ ::shutdown(fd_, SHUT_RDWR); ::close(fd_); fd_=-1; }
    }

private:
    int fd_;

    void setNonBlock(bool on){
        int f=::fcntl(fd_,F_GETFL,0);
        ::fcntl(fd_, F_SETFL, on ? f|O_NONBLOCK : f&~O_NONBLOCK);
    }

    void writeAll(const uint8_t* d, size_t n){
        size_t s=0;
        while(s<n){
            ssize_t r=::write(fd_,d+s,n-s);
            if(r<=0) throw std::runtime_error("write: "+se());
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
            ssize_t r=::read(fd_,buf+g,n-g);
            if(r==0) throw std::runtime_error("connection closed");
            if(r<0){ if(errno==EINTR) continue; throw std::runtime_error("read: "+se()); }
            g+=r;
        }
    }

    static std::string se(){ return std::string(strerror(errno)); }
};

// ─────────────────────────────────────────────────────────────────────────────
//  UnixTransportServer
// ─────────────────────────────────────────────────────────────────────────────
class UnixTransportServer : public ITransportServer {
public:
    explicit UnixTransportServer(std::string path, int backlog=16)
        : path_(std::move(path)), fd_(-1), backlog_(backlog) {}

    ~UnixTransportServer() override { close(); }

    void listen() override {
        ::unlink(path_.c_str());
        fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd_<0) throw std::runtime_error("socket: "+se());

        sockaddr_un a{}; a.sun_family=AF_UNIX;
        std::strncpy(a.sun_path, path_.c_str(), sizeof(a.sun_path)-1);
        if (::bind(fd_,(sockaddr*)&a,sizeof(a))<0)
            throw std::runtime_error("bind("+path_+"): "+se());
        if (::listen(fd_,backlog_)<0)
            throw std::runtime_error("listen: "+se());
        RIPC_DEBUG("UnixServer","Listening on "+path_);
    }

    std::unique_ptr<ITransport> accept() override {
        int cfd = ::accept(fd_, nullptr, nullptr);
        if (cfd<0) throw std::runtime_error("accept: "+se());
        return std::make_unique<UnixTransport>(cfd);
    }

    void close() override {
        if(fd_>=0){ ::close(fd_); fd_=-1; ::unlink(path_.c_str()); }
    }

    int fd() const { return fd_; }

private:
    std::string path_;
    int         fd_, backlog_;
    static std::string se(){ return std::string(strerror(errno)); }
};

} // namespace robotipc

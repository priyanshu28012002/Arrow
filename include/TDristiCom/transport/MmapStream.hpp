#pragma once
#include "IStream.hpp"
#include "../Logger.hpp"

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/eventfd.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <atomic>
#include <thread>
#include <functional>
#include <string>
#include <cstring>
#include <stdexcept>
#include <cerrno>

namespace robotipc {

// Internal shared-memory layout: atomic write-counter + payload
template<typename T>
struct MmapRegion {
    std::atomic<uint32_t> write_seq{0};  // incremented every publish
    uint8_t               pad[60]{};     // push payload to separate cache line
    T                     data{};
};

// ─────────────────────────────────────────────────────────────────────────────
//  MmapStream<T>  —  IStream over POSIX shared memory (same host only)
//  Best for: intra-machine nodes, simulation, local sensor mock
//
//  Publisher creates the region; subscribers attach.
//  Newest-wins: subscriber always reads the latest value.
// ─────────────────────────────────────────────────────────────────────────────
template<typename T>
class MmapStream : public IStream<T> {
public:
    // Publisher constructor (owner=true, creates shm)
    static std::unique_ptr<MmapStream<T>> makePublisher(const std::string& name){
        auto s = std::unique_ptr<MmapStream<T>>(new MmapStream<T>(name,true));
        return s;
    }
    // Subscriber constructor (owner=false, attaches)
    static std::unique_ptr<MmapStream<T>> makeSubscriber(const std::string& name){
        auto s = std::unique_ptr<MmapStream<T>>(new MmapStream<T>(name,false));
        return s;
    }

    ~MmapStream() override {
        unsubscribe();
        if (ptr_) ::munmap(ptr_, sizeof(MmapRegion<T>));
        if (fd_>=0) ::close(fd_);
        if (owner_) ::shm_unlink(name_.c_str());
        if (efd_>=0) ::close(efd_);
    }

    // ── Publisher ─────────────────────────────────────────────────────────────
    void publish(const T& msg) override {
        if (!ptr_) throw std::runtime_error("MmapStream not ready");
        ptr_->data = msg;
        std::atomic_thread_fence(std::memory_order_release);
        ptr_->write_seq.fetch_add(1, std::memory_order_release);
        // Signal any eventfd waiters
        if (efd_>=0){ uint64_t v=1; ::write(efd_,&v,sizeof(v)); }
    }

    // ── Subscriber ────────────────────────────────────────────────────────────
    bool latest(T& out) override {
        if (!ptr_) return false;
        std::atomic_thread_fence(std::memory_order_acquire);
        out = ptr_->data;
        return true;
    }

    void subscribe(std::function<void(const T&)> cb) override {
        cb_ = std::move(cb);
        running_.store(true);
        sub_thread_ = std::thread([this]{ subLoop(); });
    }

    void unsubscribe() override {
        running_.store(false);
        if (efd_>=0){ uint64_t v=0xFFFF; ::write(efd_,&v,sizeof(v)); }
        if (sub_thread_.joinable()) sub_thread_.join();
    }

private:
    MmapStream(const std::string& name, bool owner)
        : name_(name), owner_(owner), fd_(-1), efd_(-1), ptr_(nullptr)
    {
        size_t sz = sizeof(MmapRegion<T>);
        if (owner_) {
            ::shm_unlink(name_.c_str());
            fd_ = ::shm_open(name_.c_str(), O_CREAT|O_RDWR|O_TRUNC, 0660);
            if (fd_<0) throw std::runtime_error("shm_open create "+name_+": "+se());
            if (::ftruncate(fd_,sz)<0) throw std::runtime_error("ftruncate: "+se());
        } else {
            fd_ = ::shm_open(name_.c_str(), O_RDWR, 0660);
            if (fd_<0) throw std::runtime_error("shm_open attach "+name_+": "+se());
        }
        ptr_ = reinterpret_cast<MmapRegion<T>*>(
            ::mmap(nullptr,sz,PROT_READ|PROT_WRITE,MAP_SHARED,fd_,0));
        if (ptr_==MAP_FAILED){ ptr_=nullptr; throw std::runtime_error("mmap: "+se()); }
        if (owner_) new(ptr_) MmapRegion<T>{};  // placement-new to init atomics

        // Create eventfd for wakeup notifications
        efd_ = ::eventfd(0, EFD_NONBLOCK);
        RIPC_DEBUG("MmapStream",std::string( (owner_?"Publisher":"Subscriber"))+" "+std::string(name_));
    }

    void subLoop(){
        uint32_t last_seq = 0;
        while (running_.load()){
            // Poll eventfd with 20ms timeout (fallback polling)
            pollfd p{efd_, POLLIN, 0};
            ::poll(&p,1,20);
            uint64_t v=0; ::read(efd_,&v,sizeof(v)); // drain

            if (!running_.load()) break;
            uint32_t cur = ptr_->write_seq.load(std::memory_order_acquire);
            if (cur != last_seq){
                last_seq = cur;
                std::atomic_thread_fence(std::memory_order_acquire);
                if (cb_) cb_(ptr_->data);
            }
        }
    }

    std::string         name_;
    bool                owner_;
    int                 fd_, efd_;
    MmapRegion<T>*      ptr_;
    std::function<void(const T&)> cb_;
    std::atomic<bool>   running_{false};
    std::thread         sub_thread_;

    static std::string se(){ return std::string(strerror(errno)); }
};

} // namespace robotipc

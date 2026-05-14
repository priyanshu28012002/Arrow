#pragma once
#include "../Types.hpp"
#include "../Logger.hpp"

#include <atomic>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <string>
#include <functional>
#include <cstring>

namespace robotipc {

template<typename Feedback, typename Result>
class GoalHandle {
public:
    using Ptr = std::shared_ptr<GoalHandle<Feedback,Result>>;

    explicit GoalHandle(GoalID id): id_(id), status_(GoalStatus::ACCEPTED),
        cancel_requested_(false), result_ready_(false) {
        RIPC_DEBUG("GoalHandle","Created "+id_.str());
    }
    ~GoalHandle(){ RIPC_DEBUG("GoalHandle","Destroyed "+id_.str()); }

    // ── Worker API ────────────────────────────────────────────────────────────
    void publishFeedback(const Feedback& fb){
        std::lock_guard<std::mutex> lk(fb_mutex_);
        latest_fb_  = fb;
        fb_updated_ = true;
        if(fb_cb_) fb_cb_(fb);
    }

    bool isCancelRequested() const {
        return cancel_requested_.load(std::memory_order_acquire);
    }

    void setSucceeded(const Result& r){ transit(GoalStatus::SUCCEEDED); store(r); notify(); }
    void setAborted  (const Result& r){ transit(GoalStatus::ABORTED);   store(r); notify(); }
    void setCanceled (const Result& r=Result{}){ transit(GoalStatus::CANCELED); store(r); notify(); }

    // ── Server API ────────────────────────────────────────────────────────────
    void transitionToExecuting(){ transit(GoalStatus::EXECUTING); }

    void requestCancel(){
        auto s=status_.load();
        if(s!=GoalStatus::EXECUTING && s!=GoalStatus::ACCEPTED) return;
        transit(GoalStatus::CANCELING);
        cancel_requested_.store(true, std::memory_order_release);
        RIPC_INFO("GoalHandle",id_.str()+" cancel requested");
    }

    void setFeedbackCallback(std::function<void(const Feedback&)> cb){
        std::lock_guard<std::mutex> lk(fb_mutex_);
        fb_cb_ = std::move(cb);
    }

    // ── Query API ─────────────────────────────────────────────────────────────
    GoalStatus status() const { return status_.load(std::memory_order_acquire); }

    bool isTerminal() const {
        auto s=status();
        return s==GoalStatus::SUCCEEDED||s==GoalStatus::CANCELED||s==GoalStatus::ABORTED;
    }

    bool waitForResult(int timeout_ms=-1){
        std::unique_lock<std::mutex> lk(result_mutex_);
        if(timeout_ms<0){ result_cv_.wait(lk,[this]{return isTerminal();}); return true; }
        return result_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                                   [this]{return isTerminal();});
    }

    Result getResult() const {
        std::lock_guard<std::mutex> lk(result_mutex_);
        if(!result_ready_) throw std::runtime_error("Result not ready");
        return result_;
    }

    GoalID goalId() const { return id_; }

private:
    GoalID                   id_;
    std::atomic<GoalStatus>  status_;
    std::atomic<bool>        cancel_requested_;

    mutable std::mutex       result_mutex_;
    std::condition_variable  result_cv_;
    Result                   result_{};
    bool                     result_ready_;

    std::mutex               fb_mutex_;
    Feedback                 latest_fb_{};
    bool                     fb_updated_{false};
    std::function<void(const Feedback&)> fb_cb_;

    void transit(GoalStatus s){ status_.store(s,std::memory_order_release); }

    void store(const Result& r){
        std::lock_guard<std::mutex> lk(result_mutex_);
        result_=r; result_ready_=true;
    }

    void notify(){
        result_cv_.notify_all();
        RIPC_INFO("GoalHandle",id_.str()+" -> "+goalStatusStr(status()));
    }
};

} // namespace robotipc

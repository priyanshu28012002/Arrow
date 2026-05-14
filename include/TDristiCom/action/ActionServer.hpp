#pragma once
#include "GoalHandle.hpp"
#include "../transport/ITransport.hpp"
#include "../transport/UnixTransport.hpp"
#include "../transport/TcpTransport.hpp"
#include "../Serializer.hpp"
#include "../Logger.hpp"

#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <memory>
#include <string>
#include <cstring>

namespace robotipc {

// ── Wire structs ──────────────────────────────────────────────────────────────
#pragma pack(push,1)
struct GoalAckMsg {
    ActionMsgType type{ActionMsgType::GOAL_ACK};
    GoalID        goal_id;
    GoalStatus    status;
};
struct CancelRequestMsg {
    ActionMsgType type{ActionMsgType::CANCEL_REQUEST};
    GoalID        goal_id;
};
struct CancelAckMsg {
    ActionMsgType type{ActionMsgType::CANCEL_ACK};
    uint8_t       ok;
};
struct ResultRequestMsg {
    ActionMsgType type{ActionMsgType::RESULT_REQUEST};
    GoalID        goal_id;
};
#pragma pack(pop)

// ─────────────────────────────────────────────────────────────────────────────
//  ActionServer<Goal, Feedback, Result>
//  Three channels: <base>_goal / <base>_cancel / <base>_result
//  Factory: ActionServer::makeUnix(base) or ActionServer::makeTcp(port_base)
// ─────────────────────────────────────────────────────────────────────────────
template<typename Goal, typename Feedback, typename Result>
class ActionServer {
public:
    using Handle    = GoalHandle<Feedback,Result>;
    using HandlePtr = typename Handle::Ptr;
    using ExecFn    = std::function<void(const Goal&, HandlePtr)>;
    using CheckFn   = std::function<bool(const Goal&)>;

    // Unix factory: uses /tmp/<base>_goal, _cancel, _result
    static std::unique_ptr<ActionServer> makeUnix(const std::string& base){
        auto s = std::make_unique<ActionServer>();
        s->mode_=Mode::UNIX; s->base_=base; return s;
    }
    // TCP factory: uses ports port_base, port_base+1, port_base+2
    static std::unique_ptr<ActionServer> makeTcp(uint16_t port_base){
        auto s = std::make_unique<ActionServer>();
        s->mode_=Mode::TCP; s->port_base_=port_base; return s;
    }

    ~ActionServer(){ shutdown(); }

    void onExecute (ExecFn  f){ exec_fn_  = std::move(f); }
    void onGoalCheck(CheckFn f){ check_fn_ = std::move(f); }

    void spin(){
        if(!exec_fn_) throw std::runtime_error("No execute callback");
        startServers();
        running_.store(true);
        RIPC_INFO("ActionServer","Ready");
        std::thread ct([this]{ acceptLoop(cancel_srv_.get(), [this](auto t){ handleCancel(std::move(t)); }); });
        std::thread rt([this]{ acceptLoop(result_srv_.get(), [this](auto t){ handleResult(std::move(t)); }); });
        acceptLoop(goal_srv_.get(), [this](auto t){ handleGoal(std::move(t)); });
        ct.join(); rt.join();
    }

    void spinAsync(){
        spin_thread_ = std::thread([this]{ spin(); });
    }

    void shutdown(){
        running_.store(false);
        if(goal_srv_)   goal_srv_->close();
        if(cancel_srv_) cancel_srv_->close();
        if(result_srv_) result_srv_->close();
        if(spin_thread_.joinable()) spin_thread_.join();
    }

    ActionServer() = default;
private:
    
    enum class Mode { UNIX, TCP };

    void startServers(){
        if(mode_==Mode::UNIX){
            goal_srv_   = std::make_unique<UnixTransportServer>(base_+"_goal");
            cancel_srv_ = std::make_unique<UnixTransportServer>(base_+"_cancel");
            result_srv_ = std::make_unique<UnixTransportServer>(base_+"_result");
        } else {
            goal_srv_   = std::make_unique<TcpTransportServer>(port_base_);
            cancel_srv_ = std::make_unique<TcpTransportServer>(port_base_+1);
            result_srv_ = std::make_unique<TcpTransportServer>(port_base_+2);
        }
        goal_srv_->listen();
        cancel_srv_->listen();
        result_srv_->listen();
    }

    void acceptLoop(ITransportServer* srv,
                    std::function<void(std::unique_ptr<ITransport>)> handler){
        while(running_.load()){
            try{
                auto t = srv->accept();
                if(!running_.load()) break;
                std::thread([h=std::move(handler),t2=std::move(t)]() mutable {
                    h(std::move(t2));
                }).detach();
            } catch(const std::exception& e){
                if(running_.load()) RIPC_ERROR("ActionServer::accept",e.what());
            }
        }
    }

    void handleGoal(std::unique_ptr<ITransport> t){
        try{
            auto payload = t->recvFrame(5000);
            if(payload.empty()) return;
            ActionMsgType mt = static_cast<ActionMsgType>(payload[0]);
            if(mt!=ActionMsgType::GOAL_REQUEST) return;
            if(payload.size()<1+sizeof(Goal)){ RIPC_ERROR("ActionServer","Goal payload short"); return; }

            Goal g; std::memcpy(&g, payload.data()+1, sizeof(Goal));
            GoalID id{counter_.fetch_add(1)};

            bool ok = !check_fn_ || check_fn_(g);
            GoalAckMsg ack{}; ack.goal_id=id;
            if(!ok){
                ack.status=GoalStatus::REJECTED;
                t->sendFrame(Serializer::encodeStruct(ack));
                RIPC_WARN("ActionServer","Rejected "+id.str()); return;
            }

            auto handle = std::make_shared<Handle>(id);
            { std::lock_guard<std::mutex> lk(goals_mutex_); goals_[id.id]=handle; }

            ack.status=GoalStatus::ACCEPTED;
            t->sendFrame(Serializer::encodeStruct(ack));
            RIPC_INFO("ActionServer","Accepted "+id.str());

            std::thread([this,g,handle]() mutable {
                handle->transitionToExecuting();
                RIPC_INFO("ActionServer",handle->goalId().str()+" EXECUTING");
                try{ exec_fn_(g,handle); }
                catch(const std::exception& e){
                    RIPC_ERROR("ActionServer","Worker: "+std::string(e.what()));
                    if(!handle->isTerminal()) handle->setAborted(Result{});
                }
                std::this_thread::sleep_for(std::chrono::seconds(5));
                std::lock_guard<std::mutex> lk(goals_mutex_);
                goals_.erase(handle->goalId().id);
            }).detach();
        } catch(const std::exception& e){ RIPC_ERROR("ActionServer::goal",e.what()); }
    }

    void handleCancel(std::unique_ptr<ITransport> t){
        try{
            auto payload = t->recvFrame(3000);
            CancelRequestMsg req; std::memcpy(&req,payload.data(),sizeof(req));
            HandlePtr h;
            { std::lock_guard<std::mutex> lk(goals_mutex_);
              auto it=goals_.find(req.goal_id.id);
              if(it!=goals_.end()) h=it->second; }
            CancelAckMsg ack{};
            if(h){ h->requestCancel(); ack.ok=1; }
            else  { ack.ok=0; }
            t->sendFrame(Serializer::encodeStruct(ack));
        } catch(const std::exception& e){ RIPC_ERROR("ActionServer::cancel",e.what()); }
    }

    void handleResult(std::unique_ptr<ITransport> t){
        try{
            auto payload = t->recvFrame(3000);
            ResultRequestMsg req; std::memcpy(&req,payload.data(),sizeof(req));
            HandlePtr h;
            { std::lock_guard<std::mutex> lk(goals_mutex_);
              auto it=goals_.find(req.goal_id.id);
              if(it!=goals_.end()) h=it->second; }
            if(!h){ RIPC_WARN("ActionServer","Result: goal not found"); return; }
            h->waitForResult(60000);
            std::vector<uint8_t> rp;
            rp.push_back(uint8_t(ActionMsgType::RESULT_RESPONSE));
            rp.push_back(uint8_t(h->status()));
            Result r=h->getResult();
            const uint8_t* rptr=reinterpret_cast<const uint8_t*>(&r);
            rp.insert(rp.end(),rptr,rptr+sizeof(Result));
            t->sendFrame(Serializer::encodeVec(rp));
        } catch(const std::exception& e){ RIPC_ERROR("ActionServer::result",e.what()); }
    }

    Mode        mode_{Mode::UNIX};
    std::string base_;
    uint16_t    port_base_{0};

    std::unique_ptr<ITransportServer> goal_srv_, cancel_srv_, result_srv_;
    ExecFn   exec_fn_;
    CheckFn  check_fn_;

    std::atomic<uint64_t>                   counter_{1};
    std::mutex                              goals_mutex_;
    std::unordered_map<uint64_t,HandlePtr>  goals_;
    std::atomic<bool>                       running_{false};
    std::thread                             spin_thread_;
};

} // namespace robotipc

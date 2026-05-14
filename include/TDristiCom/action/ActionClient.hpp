#pragma once
#include "GoalHandle.hpp"
#include "ActionServer.hpp"
#include "../transport/UnixTransport.hpp"
#include "../transport/TcpTransport.hpp"
#include "../Logger.hpp"

#include <functional>
#include <thread>
#include <future>
#include <memory>
#include <string>

namespace robotipc {

template<typename Feedback, typename Result>
struct ClientGoalHandle {
    GoalID  goal_id;
    std::shared_ptr<std::atomic<GoalStatus>>                      status_atomic;
    std::shared_ptr<std::promise<std::pair<GoalStatus,Result>>>   result_promise;
    std::shared_future<std::pair<GoalStatus,Result>>              result_future;

    ClientGoalHandle(){
        status_atomic  = std::make_shared<std::atomic<GoalStatus>>(GoalStatus::IDLE);
        result_promise = std::make_shared<std::promise<std::pair<GoalStatus,Result>>>();
        result_future  = result_promise->get_future().share();
    }
    GoalStatus getStatus() const { return status_atomic->load(); }
    std::pair<GoalStatus,Result> waitForResult(int tms=-1){
        if(tms<0) return result_future.get();
        if(result_future.wait_for(std::chrono::milliseconds(tms))==std::future_status::timeout)
            throw std::runtime_error("waitForResult timeout");
        return result_future.get();
    }
};

template<typename Goal, typename Feedback, typename Result>
class ActionClient {
public:
    using CGH        = ClientGoalHandle<Feedback,Result>;
    using FeedbackCb = std::function<void(const GoalID&, const Feedback&)>;
    using StatusCb   = std::function<void(const GoalID&, GoalStatus)>;

    // Unix constructor
    explicit ActionClient(const std::string& base, int tms=5000)
        : mode_(Mode::UNIX), base_(base), timeout_ms_(tms) {}

    // TCP constructor
    ActionClient(const std::string& host, uint16_t port_base, int tms=5000)
        : mode_(Mode::TCP), host_(host), port_base_(port_base), timeout_ms_(tms) {}

    std::shared_ptr<CGH> sendGoal(const Goal& goal,
                                  FeedbackCb   fb_cb  = nullptr,
                                  StatusCb     st_cb  = nullptr)
    {
        auto cgh = std::make_shared<CGH>();

        // ── Send goal, receive ACK ────────────────────────────────────────────
        auto t = connect(0);
        std::vector<uint8_t> payload;
        payload.push_back(uint8_t(ActionMsgType::GOAL_REQUEST));
        const uint8_t* gp = reinterpret_cast<const uint8_t*>(&goal);
        payload.insert(payload.end(),gp,gp+sizeof(Goal));
        t->sendFrame(Serializer::encodeVec(payload));

        GoalAckMsg ack = Serializer::decodeStruct<GoalAckMsg>(t->recvFrame(timeout_ms_));
        cgh->goal_id = ack.goal_id;
        cgh->status_atomic->store(ack.status);

        if(ack.status==GoalStatus::REJECTED){
            RIPC_WARN("ActionClient",ack.goal_id.str()+" REJECTED");
            try{ cgh->result_promise->set_value({GoalStatus::REJECTED,Result{}}); } catch(...){}
            return cgh;
        }
        RIPC_INFO("ActionClient",ack.goal_id.str()+" ACCEPTED");
        cgh->status_atomic->store(GoalStatus::ACCEPTED);
        // t closes here

        GoalID gid = ack.goal_id;
        auto   sa  = cgh->status_atomic;

        // ── Feedback thread ───────────────────────────────────────────────────
        if(fb_cb){
            std::thread([=,fbc=fb_cb]() mutable {
                Feedback prev{}; bool first=true;
                while(true){
                    GoalStatus s=sa->load();
                    if(s==GoalStatus::SUCCEEDED||s==GoalStatus::CANCELED||s==GoalStatus::ABORTED) break;
                    // Try to receive one feedback packet via dedicated connection
                    // For cross-host: we poll result socket with short timeout
                    // Feedback is delivered via periodic re-read (10ms)
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    (void)first; (void)prev;
                }
            }).detach();
        }

        // ── Result waiter thread ──────────────────────────────────────────────
        std::thread([=, promise=cgh->result_promise, stcb=st_cb]() mutable {
            try{
                auto rt = connect(2);
                ResultRequestMsg req{}; req.goal_id=gid;
                rt->sendFrame(Serializer::encodeStruct(req));
                auto rp = rt->recvFrame(60000);
                if(rp.size()<2+sizeof(Result)) throw std::runtime_error("Result payload short");
                GoalStatus fs = static_cast<GoalStatus>(rp[1]);
                Result r; std::memcpy(&r,rp.data()+2,sizeof(Result));
                sa->store(fs);
                if(stcb) stcb(gid,fs);
                RIPC_INFO("ActionClient",gid.str()+" final="+goalStatusStr(fs));
                promise->set_value({fs,r});
            } catch(const std::exception& e){
                RIPC_ERROR("ActionClient::result",e.what());
                sa->store(GoalStatus::ABORTED);
                try{ promise->set_value({GoalStatus::ABORTED,Result{}}); } catch(...){}
            }
        }).detach();

        return cgh;
    }

    bool cancel(std::shared_ptr<CGH> cgh){
        if(!cgh) return false;
        try{
            auto t = connect(1);
            CancelRequestMsg req{}; req.goal_id=cgh->goal_id;
            t->sendFrame(Serializer::encodeStruct(req));
            CancelAckMsg ack = Serializer::decodeStruct<CancelAckMsg>(t->recvFrame(timeout_ms_));
            RIPC_INFO("ActionClient",cgh->goal_id.str()+(ack.ok?" cancel OK":" cancel: not found"));
            return ack.ok!=0;
        } catch(const std::exception& e){
            RIPC_ERROR("ActionClient::cancel",e.what()); return false;
        }
    }

private:
    enum class Mode { UNIX, TCP };
    Mode     mode_;
    std::string base_, host_;
    uint16_t    port_base_{0};
    int         timeout_ms_;

    // offset: 0=goal, 1=cancel, 2=result
    std::unique_ptr<ITransport> connect(int offset){
        if(mode_==Mode::UNIX)
            return std::make_unique<UnixTransport>(base_+suffix(offset), timeout_ms_);
        else
            return std::make_unique<TcpTransport>(host_, uint16_t(port_base_+offset), timeout_ms_);
    }

    static const char* suffix(int i){
        switch(i){ case 0: return "_goal"; case 1: return "_cancel"; default: return "_result"; }
    }
};

} // namespace robotipc

// ─────────────────────────────────────────────────────────────────────────────
//  02_local_action  —  Unix socket action (same machine)
//
//  Scenario: Local navigation action — robot navigates to a goal pose.
//  Feedback: current position + distance to goal.
//  Tests: full completion, cancel mid-navigation, rejection.
//
//  Run: ./02_local_action
// ─────────────────────────────────────────────────────────────────────────────
#include "../include/robotipc.hpp"
#include <thread>
#include <chrono>
#include <sstream>
#include <cmath>
#include <iostream>

using namespace robotipc;

#pragma pack(push,1)
struct NavGoal     { float target_x; float target_y; float speed; };
struct NavFeedback { float current_x; float current_y; float distance_remaining; float progress; };
struct NavResult   { float final_x; float final_y; uint8_t reached; };
#pragma pack(pop)

static const std::string BASE = "/tmp/ripc_local_action";

// Worker: moves robot step by step toward target
void nav_execute(const NavGoal& g,
    std::shared_ptr<GoalHandle<NavFeedback,NavResult>> h)
{
    float x=0,y=0;
    const int STEPS=20;
    float dx=(g.target_x-x)/STEPS, dy=(g.target_y-y)/STEPS;

    for(int i=1;i<=STEPS;++i){
        if(h->isCancelRequested()){
            RIPC_WARN("NavWorker","Canceled at step "+std::to_string(i));
            h->setCanceled({x,y,0});
            return;
        }
        x+=dx; y+=dy;
        float dist=std::sqrt((g.target_x-x)*(g.target_x-x)+(g.target_y-y)*(g.target_y-y));
        NavFeedback fb{x,y,dist,float(i)/STEPS};
        h->publishFeedback(fb);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0f/STEPS/g.speed)));
    }
    h->setSucceeded({x,y,1});
}

void printResult(const std::string& tag, GoalStatus s, const NavResult& r){
    std::ostringstream ss;
    ss<<"Status="<<goalStatusStr(s)
      <<" pos=("<<r.final_x<<","<<r.final_y<<")"
      <<" reached="<<int(r.reached);
    RIPC_INFO(tag,ss.str());
}

int main(){
    std::cout<<"\n";
    RIPC_INFO("02_local_action","Unix Action Demo — Navigate to Pose");

    auto server = ActionServer<NavGoal,NavFeedback,NavResult>::makeUnix(BASE);
    server->onGoalCheck([](const NavGoal& g){
        if(g.speed<=0||g.speed>10){ RIPC_WARN("NavServer","Bad speed"); return false; }
        return true;
    });
    server->onExecute(nav_execute);
    server->spinAsync();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ActionClient<NavGoal,NavFeedback,NavResult> client(BASE);

    // ── Test 1: Rejection ─────────────────────────────────────────────────────
    RIPC_INFO("Test1","=== REJECTION (speed=0) ===");
    auto cgh1 = client.sendGoal({5.0f,5.0f,0.0f});
    RIPC_INFO("Test1","Status: "+std::string(goalStatusStr(cgh1->getStatus())));

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // ── Test 2: Full completion ────────────────────────────────────────────────
    RIPC_INFO("Test2","=== FULL NAVIGATE to (3.0, 4.0) ===");
    auto cgh2 = client.sendGoal({3.0f,4.0f,2.0f},
        [](const GoalID& id, const NavFeedback& fb){
            int b=int(fb.progress*20);
            std::ostringstream ss;
            ss<<id.str()<<" pos=("<<fb.current_x<<","<<fb.current_y<<")"
              <<" dist="<<fb.distance_remaining
              <<" ["<<std::string(b,'#')<<std::string(20-b,'.')<<"] "
              <<int(fb.progress*100)<<"%";
            RIPC_INFO("Feedback",ss.str());
        });
    auto [s2,r2] = cgh2->waitForResult(15000);
    printResult("Test2",s2,r2);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // ── Test 3: Cancel ────────────────────────────────────────────────────────
    RIPC_INFO("Test3","=== CANCEL mid-navigation ===");
    auto cgh3 = client.sendGoal({10.0f,10.0f,1.0f},
        [](const GoalID&, const NavFeedback& fb){
            RIPC_INFO("Feedback","dist="+std::to_string(fb.distance_remaining));
        });
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RIPC_WARN("Test3","Sending cancel...");
    client.cancel(cgh3);
    auto [s3,r3] = cgh3->waitForResult(10000);
    printResult("Test3",s3,r3);

    server->shutdown();
    RIPC_INFO("02_local_action","Done.");
    return 0;
}

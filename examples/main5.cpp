// ─────────────────────────────────────────────────────────────────────────────
//  05_lan_action  —  TCP action over loopback (simulates LAN)
//
//  Scenario: Control PC sends robot arm trajectory goal via TCP.
//  Robot executes joint movement, sends progress feedback.
//  Tests: full execution, mid-cancel, rejection.
// ─────────────────────────────────────────────────────────────────────────────
#include "../include/robotipc.hpp"
#include <thread>
#include <chrono>
#include <sstream>
#include <cmath>
#include <iostream>

using namespace robotipc;

#pragma pack(push,1)
struct ArmGoal {
    float j1,j2,j3;    // target joint angles (degrees)
    float speed;        // 0.1 .. 1.0
};
struct ArmFeedback {
    float j1,j2,j3;    // current joint angles
    float progress;     // 0 -> 1
    float error;        // distance to target
};
struct ArmResult {
    float j1,j2,j3;
    uint8_t reached;
};
#pragma pack(pop)

static const uint16_t PORT_BASE = 7410;

void arm_execute(const ArmGoal& g,
    std::shared_ptr<GoalHandle<ArmFeedback,ArmResult>> h)
{
    const int STEPS=25;
    float cj1=0,cj2=0,cj3=0;
    float dj1=(g.j1-cj1)/STEPS, dj2=(g.j2-cj2)/STEPS, dj3=(g.j3-cj3)/STEPS;
    int delay_ms = int(50.0f/g.speed);

    for(int i=1;i<=STEPS;++i){
        if(h->isCancelRequested()){
            RIPC_WARN("ArmWorker","Canceled at "+std::to_string(i)+"/"+std::to_string(STEPS));
            h->setCanceled({cj1,cj2,cj3,0}); return;
        }
        cj1+=dj1; cj2+=dj2; cj3+=dj3;
        float err=std::sqrt((g.j1-cj1)*(g.j1-cj1)+(g.j2-cj2)*(g.j2-cj2)+(g.j3-cj3)*(g.j3-cj3));
        ArmFeedback fb{cj1,cj2,cj3,float(i)/STEPS,err};
        h->publishFeedback(fb);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    h->setSucceeded({cj1,cj2,cj3,1});
}

// ── Robot node ────────────────────────────────────────────────────────────────
void robot_node(){
    auto server = ActionServer<ArmGoal,ArmFeedback,ArmResult>::makeTcp(PORT_BASE);
    server->onGoalCheck([](const ArmGoal& g){
        if(g.speed<0.1f||g.speed>1.0f){
            RIPC_WARN("ArmServer","Bad speed="+std::to_string(g.speed)); return false;
        }
        return true;
    });
    server->onExecute(arm_execute);
    RIPC_INFO("RobotNode","Arm action server on TCP:"+std::to_string(PORT_BASE));
    server->spin();
}

// ── Control PC node ───────────────────────────────────────────────────────────
void control_node(){
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ActionClient<ArmGoal,ArmFeedback,ArmResult> client("127.0.0.1",PORT_BASE);

    // ── Rejection ─────────────────────────────────────────────────────────────
    RIPC_INFO("ControlPC","=== REJECTION (speed=0) ===");
    auto cgh1 = client.sendGoal({90,45,30,0.0f});
    RIPC_INFO("ControlPC","Status: "+std::string(goalStatusStr(cgh1->getStatus())));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // ── Full execution ────────────────────────────────────────────────────────
    RIPC_INFO("ControlPC","=== FULL ARM MOVE j1=90 j2=45 j3=30 ===");
    auto cgh2 = client.sendGoal({90,45,30,0.8f},
        [](const GoalID& id, const ArmFeedback& fb){
            int b=int(fb.progress*20);
            std::ostringstream ss;
            ss<<id.str()
              <<" J("<<fb.j1<<","<<fb.j2<<","<<fb.j3<<")"
              <<" err="<<fb.error
              <<" ["<<std::string(b,'#')<<std::string(20-b,'.')<<"] "
              <<int(fb.progress*100)<<"%";
            RIPC_INFO("Feedback",ss.str());
        });
    auto [s2,r2] = cgh2->waitForResult(20000);
    RIPC_INFO("ControlPC","Result: "+std::string(goalStatusStr(s2))
              +" reached="+std::to_string(r2.reached));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // ── Cancel mid-move ───────────────────────────────────────────────────────
    RIPC_INFO("ControlPC","=== CANCEL mid arm move ===");
    auto cgh3 = client.sendGoal({180,90,60,0.5f},
        [](const GoalID&, const ArmFeedback& fb){
            RIPC_INFO("Feedback","progress="+std::to_string(int(fb.progress*100))+"%");
        });
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    client.cancel(cgh3);
    auto [s3,r3] = cgh3->waitForResult(10000);
    RIPC_INFO("ControlPC","Result: "+std::string(goalStatusStr(s3))
              +" J("+std::to_string(r3.j1)+","+std::to_string(r3.j2)+","+std::to_string(r3.j3)+")");

    RIPC_INFO("ControlPC","Done.");
}

int main(){
    std::cout<<"\n";
    RIPC_INFO("05_lan_action","TCP Action Demo — Robot Arm Trajectory");
    auto robot = std::thread(robot_node);
    control_node();
    robot.detach();
    RIPC_INFO("05_lan_action","Done.");
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  04_lan_service  —  TCP service over loopback (simulates LAN)
//
//  Scenario: Control PC asks Robot to set motor power via TCP service.
//  Simulates two processes on LAN (loopback here).
//
//  Run: ./04_lan_service          (starts both threads in one binary)
//  Real usage: robot runs server, control PC runs client on 192.168.x.x
// ─────────────────────────────────────────────────────────────────────────────
#include "../include/robotipc.hpp"
#include <thread>
#include <sstream>
#include <vector>
#include <iostream>

using namespace robotipc;

#pragma pack(push,1)
struct MotorCmd     { int8_t left_pct; int8_t right_pct; };  // -100..100
struct MotorAck     { uint8_t ok; char msg[32]; };
#pragma pack(pop)

static const uint16_t PORT = 7401;

// ── Robot node (TCP server) ────────────────────────────────────────────────────
void robot_node(){
    auto server = ServiceServer<MotorCmd,MotorAck>::makeTcp(PORT);
    server->bind([](const MotorCmd& cmd) -> MotorAck {
        MotorAck ack{};
        if(cmd.left_pct<-100||cmd.left_pct>100||cmd.right_pct<-100||cmd.right_pct>100){
            ack.ok=0; std::snprintf(ack.msg,sizeof(ack.msg),"OUT OF RANGE");
        } else {
            ack.ok=1;
            std::snprintf(ack.msg,sizeof(ack.msg),
                "L=%d%% R=%d%%", cmd.left_pct, cmd.right_pct);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return ack;
    });
    RIPC_INFO("RobotNode","Motor service on TCP:"+std::to_string(PORT));
    server->spin();
}

// ── Control PC node (TCP client) ──────────────────────────────────────────────
void control_node(){
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    ServiceClient<MotorCmd,MotorAck> client("127.0.0.1", PORT);

    RIPC_INFO("ControlPC","--- Sequential motor commands ---");
    std::vector<std::pair<int,int>> cmds{{50,50},{-30,30},{100,100},{0,0},{150,-50}};
    for(auto& [l,r]: cmds){
        MotorCmd cmd{int8_t(l),int8_t(r)};
        MotorAck ack{};
        auto st = client.tryCall(cmd,ack,3000);
        std::ostringstream ss;
        ss<<"cmd(L="<<l<<",R="<<r<<") -> "
          <<(st==ServiceStatus::OK?(ack.ok?"OK":"REJECTED"):"FAILED")
          <<" ["<<ack.msg<<"]";
        // (st==ServiceStatus::OK&&ack.ok ? RIPC_INFO : RIPC_WARN)("ControlPC",ss.str());
    }

    RIPC_INFO("ControlPC","--- 6 concurrent motor commands ---");
    std::vector<std::thread> threads;
    for(int i=0;i<6;++i){
        threads.emplace_back([i](){
            ServiceClient<MotorCmd,MotorAck> c("127.0.0.1",PORT);
            MotorCmd cmd{int8_t(i*15),int8_t(-i*15)};
            MotorAck ack{}; c.tryCall(cmd,ack,3000);
            RIPC_INFO("ControlPC","[t"+std::to_string(i)+"] "+std::string(ack.msg));
        });
    }
    for(auto& t:threads) t.join();
    RIPC_INFO("ControlPC","Done.");
}

int main(){
    std::cout<<"\n";
    RIPC_INFO("04_lan_service","TCP Service Demo — Control PC + Robot over LAN");

    auto robot_thread = std::thread(robot_node);
    control_node();

    // Signal server to stop (connect + disconnect)
    try{ TcpTransport dummy("127.0.0.1",PORT,500); } catch(...){}
    robot_thread.detach();
    RIPC_INFO("04_lan_service","Done.");
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  03_local_stream  —  Shared-memory streams (same machine)
//
//  Scenario: Joystick publisher → robot subscriber (100 Hz)
//            IMU publisher     → dashboard subscriber (200 Hz)
//
//  Run: ./03_local_stream
// ─────────────────────────────────────────────────────────────────────────────
#include "../include/robotipc.hpp"
#include <thread>
#include <atomic>
#include <sstream>
#include <cmath>
#include <iostream>

using namespace robotipc;

#pragma pack(push,1)
struct Joystick { float lx,ly,rx,ry; uint8_t btn; };
struct IMUData  { float ax,ay,az; float gx,gy,gz; uint64_t ts_us; };
#pragma pack(pop)

int main(){
    std::cout<<"\n";
    RIPC_INFO("03_local_stream","Mmap Streams Demo — Joystick + IMU");

    // ── Joystick: publisher (simulates control PC) ────────────────────────────
    auto joy_pub = MmapStream<Joystick>::makePublisher("/ripc_joystick");
    // ── Joystick: subscriber (simulates robot) ────────────────────────────────
    auto joy_sub = MmapStream<Joystick>::makeSubscriber("/ripc_joystick");

    // ── IMU: publisher (simulates robot sensor) ───────────────────────────────
    auto imu_pub = MmapStream<IMUData>::makePublisher("/ripc_imu");
    // ── IMU: subscriber (simulates dashboard) ─────────────────────────────────
    auto imu_sub = MmapStream<IMUData>::makeSubscriber("/ripc_imu");

    std::atomic<int> joy_rx{0}, imu_rx{0};

    // ── Subscribe callbacks ────────────────────────────────────────────────────
    joy_sub->subscribe([&joy_rx](const Joystick& j){
        joy_rx.fetch_add(1);
        if(joy_rx.load()%50==0){
            std::ostringstream ss;
            ss<<"JOY lx="<<j.lx<<" ly="<<j.ly<<" btn="<<int(j.btn);
            RIPC_INFO("RobotSub",ss.str());
        }
    });

    imu_sub->subscribe([&imu_rx](const IMUData& d){
        imu_rx.fetch_add(1);
        if(imu_rx.load()%100==0){
            std::ostringstream ss;
            ss<<"IMU ax="<<d.ax<<" ay="<<d.ay<<" az="<<d.az;
            RIPC_INFO("Dashboard",ss.str());
        }
    });

    RIPC_INFO("03","Publishing for 2 seconds...");

    // ── Publish joystick at 100 Hz ────────────────────────────────────────────
    std::thread joy_thread([&](){
        for(int i=0;i<200;++i){
            float t=i*0.01f;
            Joystick j{std::sin(t), std::cos(t), 0,0, uint8_t(i%2)};
            joy_pub->publish(j);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    // ── Publish IMU at 200 Hz ─────────────────────────────────────────────────
    std::thread imu_thread([&](){
        for(int i=0;i<400;++i){
            float t=i*0.005f;
            IMUData d{t,t*0.5f,9.81f, t*0.1f,0,0, uint64_t(i*5000)};
            imu_pub->publish(d);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });

    joy_thread.join();
    imu_thread.join();

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    joy_sub->unsubscribe();
    imu_sub->unsubscribe();

    RIPC_INFO("03_local_stream","Joystick received: "+std::to_string(joy_rx.load()));
    RIPC_INFO("03_local_stream","IMU received:      "+std::to_string(imu_rx.load()));
    RIPC_INFO("03_local_stream","Done.");
    return 0;
}

// ─────────────────────────────────────────────────────────────────────────────
//  06_lan_stream  —  UDP streams over loopback (simulates LAN)
//
//  Scenario: Control PC <-> Robot over Ethernet cable
//
//  Control PC → Robot:  Joystick at 100Hz (UDP:7500)
//  Robot → Control PC:  Lidar    at  10Hz (UDP:7501)
//                       IMU      at 200Hz (UDP:7502)
//                       Odometry at  50Hz (UDP:7503)
//
//  Demonstrates: seq# drop detection, newest-wins, multi-topic
// ─────────────────────────────────────────────────────────────────────────────
#include "../include/robotipc.hpp"
#include <thread>
#include <atomic>
#include <sstream>
#include <cmath>
#include <iostream>
#include <chrono>

using namespace robotipc;

#pragma pack(push,1)
struct Joystick  { float lx,ly,rx,ry; uint8_t buttons; uint64_t ts; };
struct LidarScan { float ranges[36]; uint32_t num_pts; uint64_t ts; };
struct IMUData   { float ax,ay,az; float gx,gy,gz; float temp; uint64_t ts; };
struct Odometry  { float x,y,theta; float vx,vy,vtheta; uint64_t ts; };
#pragma pack(pop)

static uint64_t nowUs(){
    return uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

// ── Control PC side ───────────────────────────────────────────────────────────
void control_pc(){
    // Joystick publisher → robot
    auto joy_pub  = UdpStream<Joystick>::makePublisher("127.0.0.1", 7500, 0);
    // Receive robot sensors
    auto lidar_sub = UdpStream<LidarScan>::makeSubscriber(7501, 1);
    auto imu_sub   = UdpStream<IMUData>::makeSubscriber(7502, 2);
    auto odom_sub  = UdpStream<Odometry>::makeSubscriber(7503, 3);

    std::atomic<int> lidar_rx{0}, imu_rx{0}, odom_rx{0};

    lidar_sub->subscribe([&lidar_rx](const LidarScan& s){
        lidar_rx.fetch_add(1);
        if(lidar_rx.load()%5==0)
            RIPC_INFO("ControlPC:Lidar","scan pts="+std::to_string(s.num_pts)
                      +" range[0]="+std::to_string(s.ranges[0]));
    });
    imu_sub->subscribe([&imu_rx](const IMUData& d){
        imu_rx.fetch_add(1);
        if(imu_rx.load()%100==0){
            std::ostringstream ss;
            ss<<"ax="<<d.ax<<" ay="<<d.ay<<" az="<<d.az;
            RIPC_INFO("ControlPC:IMU",ss.str());
        }
    });
    odom_sub->subscribe([&odom_rx](const Odometry& o){
        odom_rx.fetch_add(1);
        if(odom_rx.load()%25==0){
            std::ostringstream ss;
            ss<<"x="<<o.x<<" y="<<o.y<<" th="<<o.theta;
            RIPC_INFO("ControlPC:Odom",ss.str());
        }
    });

    RIPC_INFO("ControlPC","Publishing joystick 100Hz for 2s...");
    for(int i=0;i<200;++i){
        float t=i*0.01f;
        Joystick j{std::sin(t),std::cos(t),0,0,uint8_t(i%4),nowUs()};
        joy_pub->publish(j);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    lidar_sub->unsubscribe();
    imu_sub->unsubscribe();
    odom_sub->unsubscribe();

    RIPC_INFO("ControlPC","Received — lidar:"+std::to_string(lidar_rx.load())
              +" imu:"+std::to_string(imu_rx.load())
              +" odom:"+std::to_string(odom_rx.load()));
}

// ── Robot side ────────────────────────────────────────────────────────────────
void robot_node(){
    // Receive joystick from control PC
    auto joy_sub   = UdpStream<Joystick>::makeSubscriber(7500, 0);
    // Publish sensors to control PC
    auto lidar_pub = UdpStream<LidarScan>::makePublisher("127.0.0.1", 7501, 1);
    auto imu_pub   = UdpStream<IMUData>::makePublisher("127.0.0.1", 7502, 2);
    auto odom_pub  = UdpStream<Odometry>::makePublisher("127.0.0.1", 7503, 3);

    std::atomic<int> joy_rx{0};
    joy_sub->subscribe([&joy_rx](const Joystick& j){
        joy_rx.fetch_add(1);
        if(joy_rx.load()%50==0)
            RIPC_INFO("Robot:Joystick","lx="+std::to_string(j.lx)
                      +" ly="+std::to_string(j.ly));
    });

    float rx=0,ry=0,rth=0;

    // Lidar at 10Hz
    std::thread lt([&](){
        for(int i=0;i<20;++i){
            LidarScan s{}; s.num_pts=36; s.ts=nowUs();
            for(int j=0;j<36;++j) s.ranges[j]=1.5f+0.5f*std::sin(j*0.174f+i*0.3f);
            lidar_pub->publish(s);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    // IMU at 200Hz
    std::thread it([&](){
        for(int i=0;i<400;++i){
            float t=i*0.005f;
            IMUData d{0.01f*std::sin(t),0.01f*std::cos(t),9.81f,
                      0.001f*t,0,0,22.5f,nowUs()};
            imu_pub->publish(d);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });

    // Odometry at 50Hz
    std::thread ot([&](){
        for(int i=0;i<100;++i){
            Joystick j{}; joy_sub->latest(j);
            rx+=j.lx*0.02f; ry+=j.ly*0.02f; rth+=j.rx*0.02f;
            Odometry o{rx,ry,rth,j.lx,j.ly,j.rx,nowUs()};
            odom_pub->publish(o);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    });

    lt.join(); it.join(); ot.join();
    joy_sub->unsubscribe();
    RIPC_INFO("Robot","Joy received: "+std::to_string(joy_rx.load()));
}

int main(){
    std::cout<<"\n";
    RIPC_INFO("06_lan_stream","UDP Stream Demo — Control PC <-> Robot (4 topics)");

    auto robot = std::thread(robot_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    control_pc();
    robot.join();

    RIPC_INFO("06_lan_stream","Done.");
    return 0;
}

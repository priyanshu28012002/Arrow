#include "../include/robotipc.hpp"
#include <thread>
#include <vector>
#include <sstream>
#include <iostream>

using namespace robotipc;

// Wire types
#pragma pack(push,1)
struct ConfigRequest  { uint8_t sensor_id; };
struct ConfigResponse { float  frequency_hz; char name[32]; uint8_t enabled; };
#pragma pack(pop)

static const std::string PATH = "/tmp/ripc_local_svc";

int main(){
    std::cout << "\n";
    RIPC_INFO("01_local_service","Unix Service Demo — same machine");

    // ── Server ────────────────────────────────────────────────────────────────
    auto server = ServiceServer<ConfigRequest,ConfigResponse>::makeUnix(PATH);
    server->bind([](const ConfigRequest& req) -> ConfigResponse {
        ConfigResponse res{};
        std::snprintf(res.name, sizeof(res.name), "Sensor_%d", req.sensor_id);
        res.frequency_hz = 10.0f * req.sensor_id;
        res.enabled      = 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return res;
    });
    server->spinAsync();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // ── Client ────────────────────────────────────────────────────────────────
    ServiceClient<ConfigRequest,ConfigResponse> client(PATH);

    RIPC_INFO("01","--- Sequential sensor config requests ---");
    for(int i=1;i<=4;++i){
        ConfigRequest req{uint8_t(i)};
        ConfigResponse res{};
        auto st = client.tryCall(req, res, 2000);
        if(st==ServiceStatus::OK){
            std::ostringstream ss;
            ss << "Sensor["<<i<<"] name="<<res.name
               <<" freq="<<res.frequency_hz<<"Hz enabled="<<int(res.enabled);
            RIPC_INFO("Client",ss.str());
        }
    }

    RIPC_INFO("01","--- 8 concurrent requests ---");
    std::vector<std::thread> threads;
    for(int i=0;i<8;++i)
        threads.emplace_back([i,&PATH](){
            ServiceClient<ConfigRequest,ConfigResponse> c(PATH);
            ConfigRequest req{uint8_t(i+1)};
            ConfigResponse res{};
            auto t0 = std::chrono::steady_clock::now();
            c.tryCall(req,res,2000);
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now()-t0).count();
            std::ostringstream ss;
            ss<<"[thread-"<<i<<"] "<<res.name<<" RTT="<<us<<"us";
            RIPC_INFO("Client",ss.str());
        });
    for(auto& t:threads) t.join();

    server->shutdown();
    RIPC_INFO("01_local_service","Done.");
    return 0;
}

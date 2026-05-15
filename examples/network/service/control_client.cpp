// control_client.cpp
// ─────────────────────────────────────────────
// Control PC (TCP Client)
// Sends motor commands to robot
// ─────────────────────────────────────────────

#include "../../../include/robotipc.hpp"
#include <thread>
#include <vector>
#include <sstream>
#include <iostream>

using namespace robotipc;

#pragma pack(push,1)

struct MotorCmd
{
    int8_t left_pct;
    int8_t right_pct;
};

struct MotorAck
{
    uint8_t ok;
    char msg[32];
};

#pragma pack(pop)

static const uint16_t PORT = 7401;

int main()
{
    // Robot IP address
    // Localhost used for demo
    ServiceClient<MotorCmd, MotorAck>
        client("127.0.0.1", PORT);

    RIPC_INFO(
        "ControlPC",
        "--- Sequential Motor Commands ---"
    );

    std::vector<std::pair<int,int>> cmds =
    {
        {50, 50},
        {-30, 30},
        {100, 100},
        {0, 0},
        {150, -50}
    };

    // Sequential requests
    for(auto& [l, r] : cmds)
    {
        MotorCmd cmd
        {
            int8_t(l),
            int8_t(r)
        };

        MotorAck ack{};

        auto st =
            client.tryCall(cmd, ack, 3000);

        std::ostringstream ss;

        ss << "cmd(L="
           << l
           << ", R="
           << r
           << ") -> ";

        if(st == ServiceStatus::OK)
        {
            ss << (ack.ok ? "OK" : "REJECTED");
        }
        else
        {
            ss << "FAILED";
        }

        ss << " [" << ack.msg << "]";

        RIPC_INFO("ControlPC", ss.str());
    }

    // ─────────────────────────────────────
    // Concurrent Requests
    // ─────────────────────────────────────

    RIPC_INFO(
        "ControlPC",
        "--- Concurrent Commands ---"
    );

    std::vector<std::thread> threads;

    for(int i = 0; i < 6; ++i)
    {
        threads.emplace_back([i]()
        {
            ServiceClient<MotorCmd, MotorAck>
                c("127.0.0.1", PORT);

            MotorCmd cmd
            {
                int8_t(i * 15),
                int8_t(-i * 15)
            };

            MotorAck ack{};

            c.tryCall(cmd, ack, 3000);

            RIPC_INFO(
                "ControlPC",
                "[t" + std::to_string(i) + "] "
                + std::string(ack.msg)
            );
        });
    }

    for(auto& t : threads)
    {
        t.join();
    }

    RIPC_INFO("ControlPC", "Done.");

    return 0;
}
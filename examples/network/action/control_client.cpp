// control_client.cpp
// ─────────────────────────────────────────────
// TCP Service Client
// Sends motor commands to robot
// ─────────────────────────────────────────────

#include "../../../include/robotipc.hpp"

#include <iostream>
#include <vector>
#include <sstream>

using namespace robotipc;

#pragma pack(push,1)

// Request packet
struct MotorCmd
{
    int8_t left_pct;
    int8_t right_pct;
};

// Response packet
struct MotorAck
{
    uint8_t ok;
    char msg[32];
};

#pragma pack(pop)

static const uint16_t PORT = 7401;

int main()
{
    // Connect to server
    ServiceClient<MotorCmd, MotorAck>
        client("192.168.0.219", PORT);

    std::vector<std::pair<int,int>> cmds =
    {
        {50, 50},
        {-30, 30},
        {100, 100},
        {0, 0},
        {150, -50}
    };

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

        RIPC_INFO(
            "ControlPC",
            ss.str()
        );
    }

    return 0;
}
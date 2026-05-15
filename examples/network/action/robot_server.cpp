// robot_server.cpp
// ─────────────────────────────────────────────
// TCP Service Server
// Robot waits for motor commands
// ─────────────────────────────────────────────

#include "../../../include/robotipc.hpp"

#include <thread>
#include <iostream>
#include <cstdio>

using namespace robotipc;

#pragma pack(push,1)

// Request packet from client
struct MotorCmd
{
    int8_t left_pct;
    int8_t right_pct;
};

// Response packet to client
struct MotorAck
{
    uint8_t ok;
    char msg[32];
};

#pragma pack(pop)

static const uint16_t PORT = 7401;

int main()
{
    // Create TCP server
    auto server =
        ServiceServer<MotorCmd, MotorAck>::makeTcp(PORT);

    // Register callback
    server->bind([](const MotorCmd& cmd) -> MotorAck
    {
        MotorAck ack{};

        // Validate motor range
        if(cmd.left_pct < -100 || cmd.left_pct > 100 ||
           cmd.right_pct < -100 || cmd.right_pct > 100)
        {
            ack.ok = 0;

            std::snprintf(
                ack.msg,
                sizeof(ack.msg),
                "OUT OF RANGE"
            );
        }
        else
        {
            ack.ok = 1;

            std::snprintf(
                ack.msg,
                sizeof(ack.msg),
                "L=%d%% R=%d%%",
                cmd.left_pct,
                cmd.right_pct
            );

            std::cout
                << "[SERVER] "
                << ack.msg
                << std::endl;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(1)
        );

        return ack;
    });

    RIPC_INFO(
        "RobotNode",
        "TCP Service running on port "
        + std::to_string(PORT)
    );

    // Infinite loop
    server->spin();

    return 0;
}
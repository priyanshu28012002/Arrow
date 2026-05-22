struct protocal
{
    int buttonId;
};
// buttonId 0000 ->
// isInt
#include <atomic>
#include <thread>
#include <mutex>
#include <string>
#include <cstdint>

#pragma pack(push, 1)
struct HeartbeatMsg
{
    uint32_t magic{0xDEADBEEF};
    uint64_t timestamp_us;
    uint8_t type;
    uint8_t pad[3]{};
};
#pragma pack(pop)


#pragma pack(push, 1)
struct MotorStatus {
	double[2] current
	double[2] voltage
	double[2] encoderPosition 
};
#pragma pack(pop)

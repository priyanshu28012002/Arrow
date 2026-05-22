#pragma once

#include <atomic>
#include <thread>
#include <mutex>
#include <string>
#include <cstdint>

enum class SessionState
{
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    RECONNECTING,
    TERMINATED
};

#pragma pack(push, 1)
struct HeartbeatMsg
{
    uint32_t magic{0xDEADBEEF};
    uint64_t timestamp_us;
    uint8_t type;
    uint8_t pad[3]{};
};
#pragma pack(pop)

class SessionManager
{
public:

    SessionManager();
    ~SessionManager();

    bool connectToServer(const std::string& ip,
                         uint16_t port);

    bool attachSocket(int socket_fd);

    void start();

    void stop();

    bool isConnected() const;

    SessionState getState() const;

private:

    void receiverLoop();

    void heartbeatLoop();

    void watchdogLoop();

    bool sendHeartbeat();

    bool sendPong();

    bool recvHeartbeat();

    bool recvAll(void* data,
                 size_t size);

    void handleDisconnect();

private:

    int socket_fd_{-1};

    std::atomic<bool> running_{false};

    std::atomic<SessionState> state_{
        SessionState::DISCONNECTED
    };

    std::atomic<uint64_t> last_rx_us_{0};

    std::thread rx_thread_;

    std::thread hb_thread_;

    std::thread watchdog_thread_;

    std::mutex socket_mutex_;
};
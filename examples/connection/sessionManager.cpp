#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <functional>
#include <memory>

#pragma pack(push, 1)
struct HeartbeatMsg
{
    uint32_t magic{0xDEADBEEF};
    uint64_t timestamp_us;
    uint8_t type; // 0=PING, 1=PONG
    uint8_t pad[3]{};
};
#pragma pack(pop)

class SessionManager
{
private:
    int isServer_;
    int socket_fd_;
    std::atomic<bool> running_;
    std::atomic<uint64_t> last_heartbeat_us_;
    std::unique_ptr<std::thread> heartbeat_thread_;
    std::unique_ptr<std::thread> watchdog_thread_;
    std::atomic<uint64_t> elapsed_ms_{0};
    
    // Reconnection parameters
    std::atomic<bool> connected_;
    std::string server_ip_;
    int port_;
    static constexpr int RECONNECT_DELAY_MS = 3000;  // 3 seconds delay between reconnection attempts

    void log(const std::string &level, const std::string &msg)
    {
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf;
        localtime_r(&t, &tm_buf);
        std::cout << std::put_time(&tm_buf, "[%H:%M:%S] ")
                  << "[" << level << "] "
                  << "[SVR:" << isServer_ << "] " << msg << "\n"
                  << std::flush;
    }

public:
    SessionManager(int isServer, int socket_fd = -1)
        : isServer_(isServer), socket_fd_(socket_fd), running_(true), connected_(false)
    {
        last_heartbeat_us_.store(nowUs());
        log("INF", "Session manager started");
    }

    ~SessionManager()
    {
        running_.store(false);
        if (heartbeat_thread_ && heartbeat_thread_->joinable())
            heartbeat_thread_->join();
        if (watchdog_thread_ && watchdog_thread_->joinable())
            watchdog_thread_->join();
        if (socket_fd_ > 0)
            close(socket_fd_);
        log("INF", "Session manager ended");
    }

    void log_info(const std::string &msg) { log("INF", msg); }
    void log_warn(const std::string &msg) { log("WRN", msg); }
    void log_err(const std::string &msg) { log("ERR", msg); }

    static uint64_t nowUs()
    {
        return uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count());
    }

    void startWatchdog()
    {
        watchdog_thread_ = std::make_unique<std::thread>([this]()
        {
            const uint64_t HEARTBEAT_TIMEOUT_US = 5000000; // 5 seconds timeout
            
            while (running_.load()) {
                uint64_t now = nowUs();
                uint64_t last = last_heartbeat_us_.load();
                elapsed_ms_.store((now - last) / 1000);
                
                // Check if heartbeat timed out and we thought we were connected
                if (connected_.load() && elapsed_ms_.load() > (HEARTBEAT_TIMEOUT_US / 1000)) {
                    log_warn("Heartbeat timeout detected!");
                    connected_.store(false);
                    
                    // Close the broken connection
                    if (socket_fd_ > 0) {
                        close(socket_fd_);
                        socket_fd_ = -1;
                    }
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }

    void startSession(int port)
    {
        if (isServer_)
        {
            log_err("startSession() only for server/robot mode");
            return;
        }
        
        port_ = port;
        log_info("Robot mode - will accept connections on port " + std::to_string(port));
        
        // Start the server acceptor thread (runs continuously)
        std::thread acceptor_thread([this]()
        {
            while (running_.load()) {
                if (!connected_.load()) {
                    log_info("Waiting for client to connect...");
                    
                    // Create server socket
                    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
                    if (server_fd < 0) {
                        log_err("socket() failed");
                        std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_DELAY_MS));
                        continue;
                    }

                    int opt = 1;
                    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

                    sockaddr_in server_addr{};
                    server_addr.sin_family = AF_INET;
                    server_addr.sin_addr.s_addr = INADDR_ANY;
                    server_addr.sin_port = htons(port_);

                    if (bind(server_fd, (sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
                        log_err("bind() failed");
                        close(server_fd);
                        std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_DELAY_MS));
                        continue;
                    }

                    if (listen(server_fd, 1) < 0) {
                        log_err("listen() failed");
                        close(server_fd);
                        std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_DELAY_MS));
                        continue;
                    }

                    // Accept new connection (blocks until client connects)
                    socket_fd_ = accept(server_fd, nullptr, nullptr);
                    if (socket_fd_ < 0) {
                        log_err("accept() failed");
                        close(server_fd);
                        std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_DELAY_MS));
                        continue;
                    }

                    log_info("Client connected!");
                    close(server_fd); // Close listening socket
                    connected_.store(true);
                    last_heartbeat_us_.store(nowUs());
                    
                    // Start heartbeat for this connection
                    startHeartbeatServer();
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
        
        acceptor_thread.detach();
        startWatchdog();
    }

    void joinSession(const std::string &server_ip, int port)
    {
        log_info("Client mode");
        if (!isServer_)
        {
            log_err("joinSession() only for client mode");
            return;
        }
        
        server_ip_ = server_ip;
        port_ = port;
        
        // Start the client reconnection thread
        std::thread client_thread([this]()
        {
            while (running_.load()) {
                if (!connected_.load()) {
                    log_info("Attempting to connect to " + server_ip_ + ":" + std::to_string(port_));
                    
                    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
                    if (socket_fd_ < 0) {
                        log_err("socket() failed");
                        std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_DELAY_MS));
                        continue;
                    }

                    sockaddr_in server_addr{};
                    server_addr.sin_family = AF_INET;
                    server_addr.sin_port = htons(port_);
                    inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr);

                    if (connect(socket_fd_, (sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
                        log_warn("Connection failed, retrying in " + 
                                std::to_string(RECONNECT_DELAY_MS/1000) + " seconds...");
                        close(socket_fd_);
                        socket_fd_ = -1;
                        std::this_thread::sleep_for(std::chrono::milliseconds(RECONNECT_DELAY_MS));
                        continue;
                    }

                    log_info("Connected to server!");
                    connected_.store(true);
                    last_heartbeat_us_.store(nowUs());
                    
                    startHeartbeatClient();
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
        
        client_thread.detach();
        startWatchdog();
    }

    bool isRunning() const { return running_; }
    bool isConnected() const { return connected_.load(); }
    uint64_t elapsedMs() const { return elapsed_ms_.load(); }
    
    void stop() { running_.store(false); }

private:
    void startHeartbeatServer()
    {
        if (heartbeat_thread_ && heartbeat_thread_->joinable())
            heartbeat_thread_->join();
            
        heartbeat_thread_ = std::make_unique<std::thread>([this]()
        {
            int hb_count = 0;
            while (running_.load() && connected_.load()) {
                HeartbeatMsg ping;
                ping.type = 0;  // PING
                ping.timestamp_us = nowUs();
                
                ssize_t n = send(socket_fd_, &ping, sizeof(ping), MSG_NOSIGNAL);
                if (n <= 0) {
                    log_err("Send failed, connection lost");
                    connected_.store(false);
                    break;
                }
                
                log_info("Sent PING #" + std::to_string(++hb_count));
                
                // Wait for PONG
                pollfd pfd{socket_fd_, POLLIN, 0};
                int rc = poll(&pfd, 1, 500);
                
                if (rc > 0 && (pfd.revents & POLLIN)) {
                    HeartbeatMsg pong;
                    ssize_t m = recv(socket_fd_, &pong, sizeof(pong), 0);
                    
                    if (m > 0 && pong.type == 1) {
                        last_heartbeat_us_.store(nowUs());
                        log_info("Received PONG #" + std::to_string(hb_count));
                    }
                } else if (rc == 0) {
                    log_warn("PONG timeout");
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            log_info("Heartbeat thread ended");
        });
    }

    void startHeartbeatClient()
    {
        if (heartbeat_thread_ && heartbeat_thread_->joinable())
            heartbeat_thread_->join();
            
        heartbeat_thread_ = std::make_unique<std::thread>([this]()
        {
            int ping_count = 0;
            while (running_.load() && connected_.load()) {
                pollfd pfd{socket_fd_, POLLIN, 0};
                int rc = poll(&pfd, 1, 1000);
                
                if (rc > 0 && (pfd.revents & POLLIN)) {
                    HeartbeatMsg msg;
                    ssize_t n = recv(socket_fd_, &msg, sizeof(msg), 0);
                    
                    if (n <= 0) {
                        log_err("Receive failed, connection lost");
                        connected_.store(false);
                        break;
                    }
                    
                    if (msg.magic == 0xDEADBEEF && msg.type == 0) {  // PING
                        last_heartbeat_us_.store(nowUs());
                        log_info("Received PING #" + std::to_string(++ping_count));
                        
                        // Send PONG
                        HeartbeatMsg pong;
                        pong.type = 1;
                        pong.timestamp_us = nowUs();
                        ssize_t m = send(socket_fd_, &pong, sizeof(pong), MSG_NOSIGNAL);
                        if (m > 0) {
                            log_info("Sent PONG #" + std::to_string(ping_count));
                        }
                    }
                } else if (rc < 0) {
                    log_err("Poll error");
                    connected_.store(false);
                    break;
                }
            }
            log_info("Heartbeat thread ended");
        });
    }
};

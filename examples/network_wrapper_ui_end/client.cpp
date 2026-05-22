#include "/home/octobot/Github/ServiceActions/Arrow/examples/connection/sessionManager.cpp"
#include <memory>

int main()
{
    SessionManager sm(1); // 1 = Client mode

    sm.joinSession("127.0.0.1", 7500);

    while (sm.isRunning())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << (int)sm.elapsedMs() << std::endl;
    }

    sm.log_info("Client session ended");
    return 0;
}
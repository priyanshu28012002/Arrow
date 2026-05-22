#include "/home/octobot/Github/ServiceActions/Arrow/examples/connection/sessionManager.cpp"
#include <memory>

int main()
{
    SessionManager sm(0); // 0 = Server/Robot mode
    sm.startSession(7500);

    while (sm.isRunning())
    {

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << (int)sm.elapsedMs() << std::endl;
    }

    sm.log_info("Robot session ended");
    return 0;
}
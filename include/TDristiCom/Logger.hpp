#pragma once
#include <iostream>
#include <sstream>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <string>

namespace robotipc {

namespace color {
    constexpr const char* RESET   = "\033[0m";
    constexpr const char* RED     = "\033[31m";
    constexpr const char* GREEN   = "\033[32m";
    constexpr const char* YELLOW  = "\033[33m";
    constexpr const char* BLUE    = "\033[34m";
    constexpr const char* MAGENTA = "\033[35m";
    constexpr const char* CYAN    = "\033[36m";
    constexpr const char* WHITE   = "\033[37m";
    constexpr const char* BOLD    = "\033[1m";
}

class Logger {
public:
    enum class Level { DEBUG=0, INFO=1, WARN=2, ERROR=3 };

    static Logger& instance() { static Logger l; return l; }
    void setLevel(Level l) { level_=l; }

    void log(Level lvl, const std::string& tag, const std::string& msg) {
        if (lvl < level_) return;
        std::lock_guard<std::mutex> lock(mtx_);

        auto now = std::chrono::system_clock::now();
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now.time_since_epoch()) % 1000;
        auto t   = std::chrono::system_clock::to_time_t(now);
        std::tm tm_b; localtime_r(&t, &tm_b);

        std::ostringstream ts;
        ts << std::put_time(&tm_b, "%H:%M:%S") << '.'
           << std::setw(3) << std::setfill('0') << ms.count();

        const char *ls, *lc;
        switch(lvl){
            case Level::DEBUG: ls="DBG"; lc=color::CYAN;    break;
            case Level::INFO:  ls="INF"; lc=color::GREEN;   break;
            case Level::WARN:  ls="WRN"; lc=color::YELLOW;  break;
            case Level::ERROR: ls="ERR"; lc=color::RED;     break;
            default:           ls="???"; lc=color::WHITE;   break;
        }
        std::cout << color::WHITE  << "[" << ts.str() << "] "
                  << lc << color::BOLD << "[" << ls << "] "
                  << color::MAGENTA    << "[" << tag << "] "
                  << color::RESET      << msg << "\n";
    }
private:
    Logger(): level_(Level::DEBUG){}
    std::mutex mtx_;
    Level      level_;
};

#define RIPC_DEBUG(tag,msg) robotipc::Logger::instance().log(robotipc::Logger::Level::DEBUG,tag,msg)
#define RIPC_INFO(tag,msg)  robotipc::Logger::instance().log(robotipc::Logger::Level::INFO, tag,msg)
#define RIPC_WARN(tag,msg)  robotipc::Logger::instance().log(robotipc::Logger::Level::WARN, tag,msg)
#define RIPC_ERROR(tag,msg) robotipc::Logger::instance().log(robotipc::Logger::Level::ERROR,tag,msg)

} // namespace robotipc

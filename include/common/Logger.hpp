/**
 * @file Logger.hpp
 * @brief Thread-Safe High-Performance Logging Utility
 */

#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <mutex>
#include <chrono>
#include <iomanip>

namespace MotorHub {

enum class LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    CRITICAL
};

class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void setLevel(LogLevel level) {
        std::lock_guard<std::mutex> lock(mutex_);
        currentLevel_ = level;
    }

    void log(LogLevel level, const std::string& tag, const std::string& message) {
        if (level < currentLevel_) return;

        auto now = std::chrono::system_clock::now();
        auto timeT = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::tm tmNow{};
#if defined(_WIN32) || defined(_WIN64)
        localtime_s(&tmNow, &timeT);
#else
        localtime_r(&timeT, &tmNow);
#endif

        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << "[" << std::put_time(&tmNow, "%H:%M:%S") << "."
                  << std::setfill('0') << std::setw(3) << ms.count() << "] ["
                  << levelToString(level) << "] ["
                  << tag << "] "
                  << message << std::endl;
    }

private:
    Logger() : currentLevel_(LogLevel::INFO) {}
    ~Logger() = default;
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    static const char* levelToString(LogLevel level) {
        switch (level) {
            case LogLevel::DEBUG:    return "DEBUG";
            case LogLevel::INFO:     return "INFO ";
            case LogLevel::WARN:     return "WARN ";
            case LogLevel::ERROR:    return "ERROR";
            case LogLevel::CRITICAL: return "CRIT ";
            default:                 return "LOG  ";
        }
    }

    std::mutex mutex_;
    LogLevel currentLevel_;
};

#define LOG_DEBUG(tag, msg) MotorHub::Logger::getInstance().log(MotorHub::LogLevel::DEBUG, tag, msg)
#define LOG_INFO(tag, msg)  MotorHub::Logger::getInstance().log(MotorHub::LogLevel::INFO, tag, msg)
#define LOG_WARN(tag, msg)  MotorHub::Logger::getInstance().log(MotorHub::LogLevel::WARN, tag, msg)
#define LOG_ERROR(tag, msg) MotorHub::Logger::getInstance().log(MotorHub::LogLevel::ERROR, tag, msg)
#define LOG_CRIT(tag, msg)  MotorHub::Logger::getInstance().log(MotorHub::LogLevel::CRITICAL, tag, msg)

} // namespace MotorHub

#endif // LOGGER_HPP

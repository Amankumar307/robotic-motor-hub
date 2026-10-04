/**
 * @file FailsafeWatchdog.hpp
 * @brief Safety Watchdog Supervising Heartbeat, Tracking Error, and Thermal Health
 */

#ifndef FAILSAFE_WATCHDOG_HPP
#define FAILSAFE_WATCHDOG_HPP

#include "core/MultiAxisController.hpp"
#include <atomic>
#include <thread>
#include <chrono>

namespace MotorHub {

class FailsafeWatchdog {
public:
    FailsafeWatchdog(MultiAxisController& controller,
                     int32_t maxTrackingErrorTicks = 30000,
                     uint32_t maxTempC = 80,
                     std::chrono::milliseconds timeoutMs = std::chrono::milliseconds(100));
    ~FailsafeWatchdog();

    void start();
    void stop();
    void kick();

    bool isTripped() const { return watchdogTripped_.load(); }
    void resetTrip() { watchdogTripped_ = false; }

private:
    void supervisionLoop();

    MultiAxisController& controller_;
    int32_t maxTrackingErrorTicks_;
    uint32_t maxTempC_;
    std::chrono::milliseconds timeoutMs_;

    std::atomic<bool> running_{false};
    std::atomic<bool> watchdogTripped_{false};
    std::atomic<int64_t> lastKickTimeMs_{0};
    std::thread monitorThread_;
};

} // namespace MotorHub

#endif // FAILSAFE_WATCHDOG_HPP

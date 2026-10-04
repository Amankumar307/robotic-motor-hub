/**
 * @file FailsafeWatchdog.cpp
 * @brief Implementation of Failsafe Watchdog
 */

#include "core/FailsafeWatchdog.hpp"
#include "common/Logger.hpp"

namespace MotorHub {

FailsafeWatchdog::FailsafeWatchdog(MultiAxisController& controller,
                                   int32_t maxTrackingErrorTicks,
                                   uint32_t maxTempC,
                                   std::chrono::milliseconds timeoutMs)
    : controller_(controller),
      maxTrackingErrorTicks_(maxTrackingErrorTicks),
      maxTempC_(maxTempC),
      timeoutMs_(timeoutMs) {}

FailsafeWatchdog::~FailsafeWatchdog() {
    stop();
}

void FailsafeWatchdog::start() {
    running_ = true;
    watchdogTripped_ = false;
    kick();
    monitorThread_ = std::thread(&FailsafeWatchdog::supervisionLoop, this);
    LOG_INFO("Watchdog", "Failsafe Watchdog active (Timeout: " + std::to_string(timeoutMs_.count()) + " ms).");
}

void FailsafeWatchdog::stop() {
    if (running_.exchange(false)) {
        if (monitorThread_.joinable()) {
            monitorThread_.join();
        }
        LOG_INFO("Watchdog", "Failsafe Watchdog stopped.");
    }
}

void FailsafeWatchdog::kick() {
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    lastKickTimeMs_ = now;
}

void FailsafeWatchdog::supervisionLoop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        // 1. Check software heartbeat timeout
        int64_t elapsedSinceKick = now - lastKickTimeMs_.load();
        if (elapsedSinceKick > timeoutMs_.count()) {
            if (!watchdogTripped_.exchange(true)) {
                LOG_CRIT("Watchdog", "HEARTBEAT TIMEOUT! Control loop missed deadline (" +
                         std::to_string(elapsedSinceKick) + " ms > " + std::to_string(timeoutMs_.count()) + " ms)!");
                controller_.globalEmergencyStop();
            }
            continue;
        }

        // 2. Check tracking errors and thermal limits
        auto telem = controller_.getSystemTelemetry();
        for (uint32_t i = 0; i < telem.activeAxes; ++i) {
            const auto& ax = telem.axes[i];

            if (std::abs(ax.trackingError) > maxTrackingErrorTicks_ && ax.state == AxisState::POSITIONING) {
                if (!watchdogTripped_.exchange(true)) {
                    LOG_CRIT("Watchdog", "EXCESSIVE TRACKING ERROR on Axis " + std::to_string(i) +
                             " (Error: " + std::to_string(ax.trackingError) + " ticks > limit: " +
                             std::to_string(maxTrackingErrorTicks_) + ")! STALL SUSPECTED.");
                    controller_.globalEmergencyStop();
                }
                break;
            }

            if (ax.temperatureC > maxTempC_) {
                if (!watchdogTripped_.exchange(true)) {
                    LOG_CRIT("Watchdog", "OVERTEMPERATURE DETECTED on Axis " + std::to_string(i) +
                             " (" + std::to_string(ax.temperatureC) + " C > limit: " +
                             std::to_string(maxTempC_) + " C)!");
                    controller_.globalEmergencyStop();
                }
                break;
            }
        }
    }
}

} // namespace MotorHub

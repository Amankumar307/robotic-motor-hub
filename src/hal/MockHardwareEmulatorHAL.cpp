/**
 * @file MockHardwareEmulatorHAL.cpp
 * @brief High-Fidelity Physics and Register Emulator HAL Implementation
 */

#include "hal/MockHardwareEmulatorHAL.hpp"
#include "common/Logger.hpp"
#include <cmath>
#include <chrono>

namespace MotorHub {

MockHardwareEmulatorHAL::MockHardwareEmulatorHAL() {
    for (size_t i = 0; i < MAX_AXES; ++i) {
        axes_[i].temperatureC = static_cast<uint32_t>(32 + i * 3);
    }
}

MockHardwareEmulatorHAL::~MockHardwareEmulatorHAL() {
    shutdown();
}

bool MockHardwareEmulatorHAL::initialize() {
    LOG_INFO("MockHAL", "Initializing Virtual Hardware Register & Physics Emulator...");
    running_ = true;
    physicsThread_ = std::thread(&MockHardwareEmulatorHAL::physicsLoop, this);
    LOG_INFO("MockHAL", "Virtual Hardware Emulator running at 1 kHz background tick rate.");
    return true;
}

void MockHardwareEmulatorHAL::shutdown() {
    if (running_.exchange(false)) {
        if (physicsThread_.joinable()) {
            physicsThread_.join();
        }
        LOG_INFO("MockHAL", "Virtual Hardware Emulator stopped cleanly.");
    }
}

void MockHardwareEmulatorHAL::physicsLoop() {
    using clock = std::chrono::steady_clock;
    constexpr auto period = std::chrono::milliseconds(1);
    auto nextTick = clock::now();

    while (running_) {
        nextTick += period;
        {
            std::lock_guard<std::mutex> lock(hwMutex_);
            tickCount_++;

            for (size_t i = 0; i < MAX_AXES; ++i) {
                auto& ax = axes_[i];

                if (estopActive_ || (ax.controlReg & ControlBits::ESTOP)) {
                    ax.pwmOutput = 0;
                    ax.continuousVel = 0.0;
                    ax.actualVel = 0;
                    ax.statusReg |= StatusBits::ESTOP_ACTIVE;
                    continue;
                }

                if (!(ax.controlReg & ControlBits::ENABLE)) {
                    ax.pwmOutput = 0;
                    ax.continuousVel = 0.0;
                    ax.actualVel = 0;
                    ax.statusReg &= ~StatusBits::ENABLED;
                    continue;
                }

                ax.statusReg |= StatusBits::ENABLED;

                // Motor physics model:
                // PWM translates to torque -> acceleration.
                // Motor terminal velocity = pwm * 100 ticks/sec.
                double targetVel = static_cast<double>(ax.pwmOutput) * 100.0;
                double accelFactor = 0.15; // Low-pass inertial response
                ax.continuousVel += accelFactor * (targetVel - ax.continuousVel);

                // Integrate position: dt = 0.001 s
                ax.continuousPos += ax.continuousVel * 0.001;

                ax.actualPos = static_cast<int32_t>(std::round(ax.continuousPos));
                ax.actualVel = static_cast<int32_t>(std::round(ax.continuousVel));

                // Check in-position condition
                if (std::abs(ax.targetPos - ax.actualPos) <= 5 && std::abs(ax.actualVel) < 5) {
                    ax.statusReg |= StatusBits::IN_POSITION;
                } else {
                    ax.statusReg &= ~StatusBits::IN_POSITION;
                }

                // Check limit switches (+/- 500,000 ticks)
                if (ax.actualPos >= 500000) {
                    ax.limitSwitches |= LimitBits::MAX_LIMIT;
                    ax.statusReg |= StatusBits::LIMIT_MAX_HIT;
                    ax.pwmOutput = 0;
                    ax.continuousVel = 0.0;
                } else if (ax.actualPos <= -500000) {
                    ax.limitSwitches |= LimitBits::MIN_LIMIT;
                    ax.statusReg |= StatusBits::LIMIT_MIN_HIT;
                    ax.pwmOutput = 0;
                    ax.continuousVel = 0.0;
                } else {
                    ax.limitSwitches = 0;
                    ax.statusReg &= ~(StatusBits::LIMIT_MIN_HIT | StatusBits::LIMIT_MAX_HIT);
                }
            }
        }
        std::this_thread::sleep_until(nextTick);
    }
}

uint32_t MockHardwareEmulatorHAL::readRegister(uint32_t axisId, uint32_t regOffset) {
    if (axisId >= MAX_AXES) return 0;
    std::lock_guard<std::mutex> lock(hwMutex_);
    const auto& ax = axes_[axisId];
    switch (regOffset) {
        case REG_OFFSET_CTRL:         return ax.controlReg;
        case REG_OFFSET_STATUS:       return ax.statusReg;
        case REG_OFFSET_TARGET_POS:   return static_cast<uint32_t>(ax.targetPos);
        case REG_OFFSET_ACTUAL_POS:   return static_cast<uint32_t>(ax.actualPos);
        case REG_OFFSET_TARGET_VEL:   return static_cast<uint32_t>(ax.targetVel);
        case REG_OFFSET_ACTUAL_VEL:   return static_cast<uint32_t>(ax.actualVel);
        case REG_OFFSET_PWM_OUTPUT:   return static_cast<uint32_t>(static_cast<int32_t>(ax.pwmOutput));
        case REG_OFFSET_KP_GAIN:      return ax.kpGain;
        case REG_OFFSET_KI_GAIN:      return ax.kiGain;
        case REG_OFFSET_KD_GAIN:      return ax.kdGain;
        case REG_OFFSET_LIMIT_SWITCH: return ax.limitSwitches;
        case REG_OFFSET_IRQ_STATUS:   return ax.irqStatus;
        default:                      return 0;
    }
}

void MockHardwareEmulatorHAL::writeRegister(uint32_t axisId, uint32_t regOffset, uint32_t value) {
    if (axisId >= MAX_AXES) return;
    std::lock_guard<std::mutex> lock(hwMutex_);
    auto& ax = axes_[axisId];
    switch (regOffset) {
        case REG_OFFSET_CTRL:
            ax.controlReg = value;
            if (value & ControlBits::RESET) {
                ax.continuousPos = 0.0;
                ax.continuousVel = 0.0;
                ax.actualPos = 0;
                ax.actualVel = 0;
                ax.targetPos = 0;
                ax.targetVel = 0;
                ax.statusReg = StatusBits::ENABLED | StatusBits::IN_POSITION;
            }
            break;
        case REG_OFFSET_TARGET_POS: ax.targetPos = static_cast<int32_t>(value); break;
        case REG_OFFSET_TARGET_VEL: ax.targetVel = static_cast<int32_t>(value); break;
        case REG_OFFSET_PWM_OUTPUT:
            if (!estopActive_) ax.pwmOutput = static_cast<int16_t>(value);
            break;
        case REG_OFFSET_KP_GAIN:    ax.kpGain = value; break;
        case REG_OFFSET_KI_GAIN:    ax.kiGain = value; break;
        case REG_OFFSET_KD_GAIN:    ax.kdGain = value; break;
        default: break;
    }
}

bool MockHardwareEmulatorHAL::setMotorCommand(uint32_t axisId, int32_t targetPos, int32_t targetVel, int16_t pwmEffort, uint16_t flags) {
    if (axisId >= MAX_AXES) return false;
    std::lock_guard<std::mutex> lock(hwMutex_);
    if (estopActive_) return false;

    auto& ax = axes_[axisId];
    ax.targetPos = targetPos;
    ax.targetVel = targetVel;
    ax.pwmOutput = pwmEffort;
    if (flags & ControlBits::ENABLE) {
        ax.controlReg |= ControlBits::ENABLE;
    }
    return true;
}

bool MockHardwareEmulatorHAL::getAxisStatus(uint32_t axisId, AxisTelemetry& outTelemetry) {
    if (axisId >= MAX_AXES) return false;
    std::lock_guard<std::mutex> lock(hwMutex_);
    const auto& ax = axes_[axisId];

    outTelemetry.axisId = axisId;
    outTelemetry.actualPosition = ax.actualPos;
    outTelemetry.actualVelocity = ax.actualVel;
    outTelemetry.targetPosition = ax.targetPos;
    outTelemetry.targetVelocity = ax.targetVel;
    outTelemetry.pwmEffort = ax.pwmOutput;
    outTelemetry.statusFlags = static_cast<uint16_t>(ax.statusReg);
    outTelemetry.trackingError = ax.targetPos - ax.actualPos;
    outTelemetry.temperatureC = ax.temperatureC;
    return true;
}

bool MockHardwareEmulatorHAL::readAllTelemetry(SystemTelemetryPacket& outPacket) {
    std::lock_guard<std::mutex> lock(hwMutex_);
    outPacket.sequenceNumber = tickCount_;
    outPacket.timestampNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    outPacket.activeAxes = MAX_AXES;
    outPacket.emergencyStopActive = estopActive_;

    for (size_t i = 0; i < MAX_AXES; ++i) {
        const auto& ax = axes_[i];
        outPacket.axes[i].axisId = static_cast<uint32_t>(i);
        outPacket.axes[i].targetPosition = ax.targetPos;
        outPacket.axes[i].actualPosition = ax.actualPos;
        outPacket.axes[i].targetVelocity = ax.targetVel;
        outPacket.axes[i].actualVelocity = ax.actualVel;
        outPacket.axes[i].pwmEffort = ax.pwmOutput;
        outPacket.axes[i].statusFlags = static_cast<uint16_t>(ax.statusReg);
        outPacket.axes[i].trackingError = ax.targetPos - ax.actualPos;
        outPacket.axes[i].temperatureC = ax.temperatureC;
    }
    return true;
}

void MockHardwareEmulatorHAL::triggerEmergencyStop() {
    estopActive_ = true;
    std::lock_guard<std::mutex> lock(hwMutex_);
    for (size_t i = 0; i < MAX_AXES; ++i) {
        axes_[i].pwmOutput = 0;
        axes_[i].continuousVel = 0.0;
        axes_[i].statusReg |= StatusBits::ESTOP_ACTIVE;
    }
    LOG_CRIT("MockHAL", "Hardware Emergency Stop TRIGGERED in Virtual Emulator!");
}

void MockHardwareEmulatorHAL::clearEmergencyStop() {
    estopActive_ = false;
    std::lock_guard<std::mutex> lock(hwMutex_);
    for (size_t i = 0; i < MAX_AXES; ++i) {
        axes_[i].statusReg &= ~StatusBits::ESTOP_ACTIVE;
    }
    LOG_INFO("MockHAL", "Emergency Stop cleared in Virtual Emulator.");
}

bool MockHardwareEmulatorHAL::isEmergencyStopActive() const {
    return estopActive_.load();
}

void MockHardwareEmulatorHAL::injectLimitFault(uint32_t axisId, bool maxLimit) {
    if (axisId >= MAX_AXES) return;
    std::lock_guard<std::mutex> lock(hwMutex_);
    if (maxLimit) {
        axes_[axisId].limitSwitches |= LimitBits::MAX_LIMIT;
        axes_[axisId].statusReg |= StatusBits::LIMIT_MAX_HIT;
    } else {
        axes_[axisId].limitSwitches |= LimitBits::MIN_LIMIT;
        axes_[axisId].statusReg |= StatusBits::LIMIT_MIN_HIT;
    }
}

void MockHardwareEmulatorHAL::resetFaults(uint32_t axisId) {
    if (axisId >= MAX_AXES) return;
    std::lock_guard<std::mutex> lock(hwMutex_);
    axes_[axisId].limitSwitches = 0;
    axes_[axisId].statusReg &= ~(StatusBits::LIMIT_MIN_HIT | StatusBits::LIMIT_MAX_HIT | StatusBits::ERROR);
}

} // namespace MotorHub

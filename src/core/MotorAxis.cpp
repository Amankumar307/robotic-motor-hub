/**
 * @file MotorAxis.cpp
 * @brief Implementation of Motor Axis Controller and State Machine
 */

#include "core/MotorAxis.hpp"
#include "common/Logger.hpp"
#include <cmath>
#include <algorithm>

namespace MotorHub {

MotorAxis::MotorAxis(const AxisConfig& config, IHardwareDevice& hal)
    : config_(config), hal_(hal), pid_(config.pid) {
    transitionTo(AxisState::IDLE);
}

void MotorAxis::transitionTo(AxisState newState) {
    if (state_ == newState) return;
    LOG_INFO(config_.name, "State transition: " + std::string(toString(state_)) + " -> " + std::string(toString(newState)));
    state_ = newState;
}

void MotorAxis::setEnabled(bool enable) {
    if (state_ == AxisState::EMERGENCY_STOP) {
        LOG_WARN(config_.name, "Cannot enable while in EMERGENCY_STOP!");
        return;
    }
    if (enable) {
        transitionTo(AxisState::IDLE);
        hal_.writeRegister(config_.axisId, REG_OFFSET_CTRL, ControlBits::ENABLE);
    } else {
        transitionTo(AxisState::UNINITIALIZED);
        hal_.writeRegister(config_.axisId, REG_OFFSET_CTRL, 0);
    }
}

void MotorAxis::emergencyStop() {
    transitionTo(AxisState::EMERGENCY_STOP);
    trajectoryActive_ = false;
    lastPwmEffort_ = 0;
    hal_.setMotorCommand(config_.axisId, actualPosition_, 0, 0, ControlBits::ESTOP);
}

bool MotorAxis::resetFault() {
    if (state_ != AxisState::ERROR && state_ != AxisState::EMERGENCY_STOP) {
        return false;
    }
    LOG_INFO(config_.name, "Fault reset requested.");
    hal_.writeRegister(config_.axisId, REG_OFFSET_CTRL, ControlBits::RESET | ControlBits::ENABLE);
    pid_.reset();
    transitionTo(AxisState::IDLE);
    return true;
}

bool MotorAxis::moveTo(int32_t targetPosition) {
    if (state_ == AxisState::EMERGENCY_STOP || state_ == AxisState::ERROR || state_ == AxisState::UNINITIALIZED) {
        LOG_WARN(config_.name, "Cannot move: axis in " + std::string(toString(state_)) + " state.");
        return false;
    }

    if (targetPosition < config_.minLimitTicks || targetPosition > config_.maxLimitTicks) {
        LOG_ERROR(config_.name, "Target position " + std::to_string(targetPosition) + " exceeds soft limits!");
        return false;
    }

    activeProfile_ = TrajectoryPlanner::generateProfile(
        static_cast<double>(actualPosition_),
        static_cast<double>(targetPosition),
        static_cast<double>(config_.maxVelocityTicksPerSec),
        static_cast<double>(config_.maxAccelerationTicksPerSec2)
    );

    trajectoryElapsedSec_ = 0.0;
    trajectoryActive_ = true;
    targetPosition_ = targetPosition;
    transitionTo(AxisState::POSITIONING);
    return true;
}

bool MotorAxis::setVelocity(int32_t targetVelocity) {
    if (state_ == AxisState::EMERGENCY_STOP || state_ == AxisState::ERROR) {
        return false;
    }
    targetVelocity_ = std::clamp(targetVelocity, -config_.maxVelocityTicksPerSec, config_.maxVelocityTicksPerSec);
    trajectoryActive_ = false;
    transitionTo(AxisState::JOGGING);
    return true;
}

bool MotorAxis::home() {
    if (state_ == AxisState::EMERGENCY_STOP || state_ == AxisState::ERROR) {
        return false;
    }
    transitionTo(AxisState::HOMING);
    LOG_INFO(config_.name, "Initiating homing sequence...");
    // Direct command to home register
    hal_.writeRegister(config_.axisId, REG_OFFSET_CTRL, ControlBits::HOME | ControlBits::ENABLE);
    isHomed_ = true;
    actualPosition_ = 0;
    targetPosition_ = 0;
    transitionTo(AxisState::IDLE);
    return true;
}

void MotorAxis::setSynchronizedProfile(const TrapezoidalProfile& profile) {
    activeProfile_ = profile;
    trajectoryElapsedSec_ = 0.0;
    trajectoryActive_ = true;
    targetPosition_ = static_cast<int32_t>(profile.targetPos);
    transitionTo(AxisState::POSITIONING);
}

void MotorAxis::update(double dt) {
    // Read hardware status via HAL
    AxisTelemetry telem{};
    if (hal_.getAxisStatus(config_.axisId, telem)) {
        actualPosition_ = telem.actualPosition;
        actualVelocity_ = telem.actualVelocity;
        statusFlags_ = telem.statusFlags;
        temperatureC_ = telem.temperatureC;
    }

    if (state_ == AxisState::EMERGENCY_STOP) {
        hal_.setMotorCommand(config_.axisId, actualPosition_, 0, 0, ControlBits::ESTOP);
        return;
    }

    // Check hardware limit switch strike
    if (statusFlags_ & (StatusBits::LIMIT_MIN_HIT | StatusBits::LIMIT_MAX_HIT)) {
        LOG_CRIT(config_.name, "Hardware limit switch hit! Tripping axis error.");
        transitionTo(AxisState::ERROR);
        hal_.setMotorCommand(config_.axisId, actualPosition_, 0, 0, 0);
        return;
    }

    if (trajectoryActive_) {
        trajectoryElapsedSec_ += dt;
        auto waypoint = TrajectoryPlanner::sampleProfile(activeProfile_, trajectoryElapsedSec_);
        targetPosition_ = static_cast<int32_t>(waypoint.position);
        targetVelocity_ = static_cast<int32_t>(waypoint.velocity);

        if (trajectoryElapsedSec_ >= activeProfile_.totalDuration) {
            trajectoryActive_ = false;
        }
    }

    // Closed-loop PID position control
    double pidEffort = pid_.compute(static_cast<double>(targetPosition_), static_cast<double>(actualPosition_), dt);
    int16_t pwmOutput = static_cast<int16_t>(std::clamp(pidEffort, -1000.0, 1000.0));
    lastPwmEffort_ = pwmOutput;

    // Send updated command to hardware/driver
    hal_.setMotorCommand(config_.axisId, targetPosition_, targetVelocity_, pwmOutput, ControlBits::ENABLE);

    // State convergence check
    if (state_ == AxisState::POSITIONING && !trajectoryActive_) {
        int32_t posError = std::abs(targetPosition_ - actualPosition_);
        if (posError <= config_.positionDeadbandTicks && std::abs(actualVelocity_) < 5) {
            transitionTo(AxisState::IDLE);
        }
    }
}

AxisTelemetry MotorAxis::getTelemetry() const {
    AxisTelemetry telem;
    telem.axisId = config_.axisId;
    telem.state = state_;
    telem.targetPosition = targetPosition_;
    telem.actualPosition = actualPosition_;
    telem.trackingError = targetPosition_ - actualPosition_;
    telem.targetVelocity = targetVelocity_;
    telem.actualVelocity = actualVelocity_;
    telem.pwmEffort = lastPwmEffort_;
    telem.statusFlags = statusFlags_;
    telem.temperatureC = temperatureC_;
    return telem;
}

void MotorAxis::setPIDParameters(const PIDParameters& params) {
    config_.pid = params;
    pid_.setParameters(params);
}

} // namespace MotorHub

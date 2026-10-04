/**
 * @file MultiAxisController.cpp
 * @brief Implementation of Multi-Axis Motion Controller
 */

#include "core/MultiAxisController.hpp"
#include "common/Logger.hpp"
#include <chrono>

namespace MotorHub {

MultiAxisController::MultiAxisController(IHardwareDevice& hal)
    : hal_(hal) {}

bool MultiAxisController::initializeAxes(const std::vector<AxisConfig>& configs) {
    std::lock_guard<std::mutex> lock(controllerMutex_);
    axes_.clear();
    for (const auto& cfg : configs) {
        axes_.push_back(std::make_unique<MotorAxis>(cfg, hal_));
    }
    LOG_INFO("MultiAxis", "Initialized " + std::to_string(axes_.size()) + " coordinated motion axes.");
    return true;
}

void MultiAxisController::updateControlLoop(double dt) {
    std::lock_guard<std::mutex> lock(controllerMutex_);
    loopCounter_++;

    for (auto& axis : axes_) {
        axis->update(dt);
    }
}

bool MultiAxisController::moveSingleAxis(uint32_t axisId, int32_t targetPosition) {
    std::lock_guard<std::mutex> lock(controllerMutex_);
    if (globalEstopActive_ || axisId >= axes_.size()) return false;
    return axes_[axisId]->moveTo(targetPosition);
}

bool MultiAxisController::moveSynchronized(const std::array<int32_t, MAX_AXES>& targetPositions) {
    std::lock_guard<std::mutex> lock(controllerMutex_);
    if (globalEstopActive_) return false;

    std::array<double, MAX_AXES> startPos{};
    std::array<double, MAX_AXES> targets{};
    std::array<double, MAX_AXES> maxVels{};
    std::array<double, MAX_AXES> maxAccels{};

    for (size_t i = 0; i < axes_.size(); ++i) {
        auto telem = axes_[i]->getTelemetry();
        startPos[i] = static_cast<double>(telem.actualPosition);
        targets[i] = static_cast<double>(targetPositions[i]);
        maxVels[i] = static_cast<double>(axes_[i]->getConfig().maxVelocityTicksPerSec);
        maxAccels[i] = static_cast<double>(axes_[i]->getConfig().maxAccelerationTicksPerSec2);
    }

    auto profiles = TrajectoryPlanner::generateSynchronizedProfiles(startPos, targets, maxVels, maxAccels);

    for (size_t i = 0; i < axes_.size(); ++i) {
        axes_[i]->setSynchronizedProfile(profiles[i]);
    }

    LOG_INFO("MultiAxis", "Dispatched synchronized multi-axis trajectory across " + std::to_string(axes_.size()) + " axes.");
    return true;
}

bool MultiAxisController::homeAxis(uint32_t axisId) {
    std::lock_guard<std::mutex> lock(controllerMutex_);
    if (axisId >= axes_.size()) return false;
    return axes_[axisId]->home();
}

bool MultiAxisController::homeAllAxes() {
    std::lock_guard<std::mutex> lock(controllerMutex_);
    bool success = true;
    for (auto& ax : axes_) {
        if (!ax->home()) success = false;
    }
    return success;
}

void MultiAxisController::globalEmergencyStop() {
    std::lock_guard<std::mutex> lock(controllerMutex_);
    globalEstopActive_ = true;
    hal_.triggerEmergencyStop();
    for (auto& ax : axes_) {
        ax->emergencyStop();
    }
    LOG_CRIT("MultiAxis", "GLOBAL EMERGENCY STOP executed across all axes!");
}

void MultiAxisController::clearEmergencyStop() {
    std::lock_guard<std::mutex> lock(controllerMutex_);
    hal_.clearEmergencyStop();
    globalEstopActive_ = false;
    for (auto& ax : axes_) {
        ax->resetFault();
    }
    LOG_INFO("MultiAxis", "Global Emergency Stop cleared. All axes re-armed to IDLE.");
}

bool MultiAxisController::resetAxisFault(uint32_t axisId) {
    std::lock_guard<std::mutex> lock(controllerMutex_);
    if (axisId >= axes_.size()) return false;
    return axes_[axisId]->resetFault();
}

SystemTelemetryPacket MultiAxisController::getSystemTelemetry() {
    std::lock_guard<std::mutex> lock(controllerMutex_);
    SystemTelemetryPacket packet{};
    packet.sequenceNumber = loopCounter_;
    packet.timestampNs = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    packet.activeAxes = static_cast<uint32_t>(axes_.size());
    packet.emergencyStopActive = globalEstopActive_ || hal_.isEmergencyStopActive();

    for (size_t i = 0; i < axes_.size(); ++i) {
        packet.axes[i] = axes_[i]->getTelemetry();
    }
    return packet;
}

MotorAxis* MultiAxisController::getAxis(uint32_t axisId) {
    std::lock_guard<std::mutex> lock(controllerMutex_);
    if (axisId >= axes_.size()) return nullptr;
    return axes_[axisId].get();
}

} // namespace MotorHub

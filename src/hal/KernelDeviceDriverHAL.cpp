/**
 * @file KernelDeviceDriverHAL.cpp
 * @brief Implementation of Linux Kernel Driver HAL
 */

#include "hal/KernelDeviceDriverHAL.hpp"
#include "common/Logger.hpp"

#if defined(__linux__)
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstring>
#include "../../driver/motor_hub_uapi.h"
#endif

namespace MotorHub {

KernelDeviceDriverHAL::KernelDeviceDriverHAL(std::string devicePath)
    : devicePath_(std::move(devicePath)), deviceFd_(-1), estopActive_(false) {}

KernelDeviceDriverHAL::~KernelDeviceDriverHAL() {
    shutdown();
}

bool KernelDeviceDriverHAL::initialize() {
#if defined(__linux__)
    deviceFd_ = ::open(devicePath_.c_str(), O_RDWR);
    if (deviceFd_ < 0) {
        LOG_ERROR("KernelHAL", "Failed to open kernel device: " + devicePath_ + " (errno: " + std::to_string(errno) + ")");
        return false;
    }
    LOG_INFO("KernelHAL", "Successfully opened Linux kernel device: " + devicePath_ + " [fd: " + std::to_string(deviceFd_) + "]");
    return true;
#else
    LOG_WARN("KernelHAL", "Linux kernel device /dev/motor_hub is only available on native Linux/WSL2.");
    return false;
#endif
}

void KernelDeviceDriverHAL::shutdown() {
#if defined(__linux__)
    if (deviceFd_ >= 0) {
        ::close(deviceFd_);
        deviceFd_ = -1;
        LOG_INFO("KernelHAL", "Closed Linux kernel device: " + devicePath_);
    }
#endif
}

uint32_t KernelDeviceDriverHAL::readRegister(uint32_t axisId, uint32_t regOffset) {
#if defined(__linux__)
    if (deviceFd_ < 0) return 0;
    struct motor_reg_access reg{};
    reg.axis_id = axisId;
    reg.reg_offset = regOffset;
    if (::ioctl(deviceFd_, IOCTL_MOTOR_READ_REG, &reg) < 0) {
        LOG_ERROR("KernelHAL", "IOCTL_MOTOR_READ_REG failed for axis " + std::to_string(axisId));
        return 0;
    }
    return reg.value;
#else
    (void)axisId; (void)regOffset;
    return 0;
#endif
}

void KernelDeviceDriverHAL::writeRegister(uint32_t axisId, uint32_t regOffset, uint32_t value) {
#if defined(__linux__)
    if (deviceFd_ < 0) return;
    struct motor_reg_access reg{};
    reg.axis_id = axisId;
    reg.reg_offset = regOffset;
    reg.value = value;
    if (::ioctl(deviceFd_, IOCTL_MOTOR_WRITE_REG, &reg) < 0) {
        LOG_ERROR("KernelHAL", "IOCTL_MOTOR_WRITE_REG failed for axis " + std::to_string(axisId));
    }
#else
    (void)axisId; (void)regOffset; (void)value;
#endif
}

bool KernelDeviceDriverHAL::setMotorCommand(uint32_t axisId, int32_t targetPos, int32_t targetVel, int16_t pwmEffort, uint16_t flags) {
#if defined(__linux__)
    if (deviceFd_ < 0) return false;
    struct motor_axis_command cmd{};
    cmd.axis_id = axisId;
    cmd.target_pos = targetPos;
    cmd.target_vel = targetVel;
    cmd.pwm_effort = pwmEffort;
    cmd.control_flags = flags;
    return (::ioctl(deviceFd_, IOCTL_MOTOR_SET_CMD, &cmd) == 0);
#else
    (void)axisId; (void)targetPos; (void)targetVel; (void)pwmEffort; (void)flags;
    return false;
#endif
}

bool KernelDeviceDriverHAL::getAxisStatus(uint32_t axisId, AxisTelemetry& outTelemetry) {
#if defined(__linux__)
    if (deviceFd_ < 0) return false;
    struct motor_axis_status_report rep{};
    rep.axis_id = axisId;
    if (::ioctl(deviceFd_, IOCTL_MOTOR_GET_STATUS, &rep) < 0) return false;

    outTelemetry.axisId = axisId;
    outTelemetry.actualPosition = rep.actual_pos;
    outTelemetry.actualVelocity = rep.actual_vel;
    outTelemetry.pwmEffort = rep.current_pwm;
    outTelemetry.statusFlags = rep.status_flags;
    outTelemetry.trackingError = rep.tracking_error;
    outTelemetry.temperatureC = rep.temperature_c;
    return true;
#else
    (void)axisId; (void)outTelemetry;
    return false;
#endif
}

bool KernelDeviceDriverHAL::readAllTelemetry(SystemTelemetryPacket& outPacket) {
#if defined(__linux__)
    if (deviceFd_ < 0) return false;
    struct motor_hub_telemetry_batch batch{};
    if (::ioctl(deviceFd_, IOCTL_MOTOR_READ_ALL, &batch) < 0) return false;

    outPacket.timestampNs = batch.timestamp_ns;
    outPacket.activeAxes = batch.active_axes;
    outPacket.emergencyStopActive = (batch.global_status != 0);

    for (size_t i = 0; i < MAX_AXES && i < batch.active_axes; i++) {
        outPacket.axes[i].axisId = batch.axes[i].axis_id;
        outPacket.axes[i].actualPosition = batch.axes[i].actual_pos;
        outPacket.axes[i].actualVelocity = batch.axes[i].actual_vel;
        outPacket.axes[i].pwmEffort = batch.axes[i].current_pwm;
        outPacket.axes[i].statusFlags = batch.axes[i].status_flags;
        outPacket.axes[i].trackingError = batch.axes[i].tracking_error;
        outPacket.axes[i].temperatureC = batch.axes[i].temperature_c;
    }
    return true;
#else
    (void)outPacket;
    return false;
#endif
}

void KernelDeviceDriverHAL::triggerEmergencyStop() {
    estopActive_ = true;
#if defined(__linux__)
    if (deviceFd_ >= 0) {
        ::ioctl(deviceFd_, IOCTL_MOTOR_TRIGGER_ESTOP);
    }
#endif
    LOG_CRIT("KernelHAL", "Emergency stop triggered via Kernel HAL!");
}

void KernelDeviceDriverHAL::clearEmergencyStop() {
    estopActive_ = false;
#if defined(__linux__)
    if (deviceFd_ >= 0) {
        ::ioctl(deviceFd_, IOCTL_MOTOR_CLEAR_ESTOP);
    }
#endif
    LOG_INFO("KernelHAL", "Emergency stop cleared via Kernel HAL.");
}

bool KernelDeviceDriverHAL::isEmergencyStopActive() const {
    return estopActive_.load();
}

} // namespace MotorHub

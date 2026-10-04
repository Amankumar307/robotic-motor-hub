/**
 * @file KernelDeviceDriverHAL.hpp
 * @brief Production Linux Kernel Character Device Driver HAL
 */

#ifndef KERNEL_DEVICE_DRIVER_HAL_HPP
#define KERNEL_DEVICE_DRIVER_HAL_HPP

#include "hal/IHardwareDevice.hpp"
#include <string>
#include <atomic>

namespace MotorHub {

class KernelDeviceDriverHAL : public IHardwareDevice {
public:
    explicit KernelDeviceDriverHAL(std::string devicePath = "/dev/motor_hub");
    ~KernelDeviceDriverHAL() override;

    bool initialize() override;
    void shutdown() override;

    uint32_t readRegister(uint32_t axisId, uint32_t regOffset) override;
    void writeRegister(uint32_t axisId, uint32_t regOffset, uint32_t value) override;

    bool setMotorCommand(uint32_t axisId, int32_t targetPos, int32_t targetVel, int16_t pwmEffort, uint16_t flags) override;
    bool getAxisStatus(uint32_t axisId, AxisTelemetry& outTelemetry) override;
    bool readAllTelemetry(SystemTelemetryPacket& outPacket) override;

    void triggerEmergencyStop() override;
    void clearEmergencyStop() override;
    bool isEmergencyStopActive() const override;

private:
    std::string devicePath_;
    int deviceFd_{-1};
    std::atomic<bool> estopActive_{false};
};

} // namespace MotorHub

#endif // KERNEL_DEVICE_DRIVER_HAL_HPP

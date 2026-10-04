/**
 * @file IHardwareDevice.hpp
 * @brief Abstract Interface for Motor Controller Hardware / Emulation Layer
 */

#ifndef I_HARDWARE_DEVICE_HPP
#define I_HARDWARE_DEVICE_HPP

#include "common/MotorTypes.hpp"
#include "common/HardwareRegisters.hpp"
#include <cstdint>

namespace MotorHub {

class IHardwareDevice {
public:
    virtual ~IHardwareDevice() = default;

    virtual bool initialize() = 0;
    virtual void shutdown() = 0;

    virtual uint32_t readRegister(uint32_t axisId, uint32_t regOffset) = 0;
    virtual void writeRegister(uint32_t axisId, uint32_t regOffset, uint32_t value) = 0;

    virtual bool setMotorCommand(uint32_t axisId, int32_t targetPos, int32_t targetVel, int16_t pwmEffort, uint16_t flags) = 0;
    virtual bool getAxisStatus(uint32_t axisId, AxisTelemetry& outTelemetry) = 0;
    virtual bool readAllTelemetry(SystemTelemetryPacket& outPacket) = 0;

    virtual void triggerEmergencyStop() = 0;
    virtual void clearEmergencyStop() = 0;
    virtual bool isEmergencyStopActive() const = 0;
};

} // namespace MotorHub

#endif // I_HARDWARE_DEVICE_HPP

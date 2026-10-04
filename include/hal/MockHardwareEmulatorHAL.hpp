/**
 * @file MockHardwareEmulatorHAL.hpp
 * @brief High-Fidelity Physics and Register Emulator HAL
 */

#ifndef MOCK_HARDWARE_EMULATOR_HAL_HPP
#define MOCK_HARDWARE_EMULATOR_HAL_HPP

#include "hal/IHardwareDevice.hpp"
#include <array>
#include <atomic>
#include <thread>
#include <mutex>

namespace MotorHub {

struct SimulatedAxisHardware {
    uint32_t controlReg{ControlBits::ENABLE};
    uint32_t statusReg{StatusBits::ENABLED};
    int32_t  targetPos{0};
    int32_t  actualPos{0};
    int32_t  targetVel{0};
    int32_t  actualVel{0};
    int16_t  pwmOutput{0};
    uint32_t kpGain{0x00010000}; // 1.0 in Q16.16
    uint32_t kiGain{0x00004000}; // 0.25 in Q16.16
    uint32_t kdGain{0x00002000}; // 0.125 in Q16.16
    uint32_t limitSwitches{0};
    uint32_t irqStatus{0};
    uint32_t temperatureC{36};

    double   continuousPos{0.0};
    double   continuousVel{0.0};
};

class MockHardwareEmulatorHAL : public IHardwareDevice {
public:
    MockHardwareEmulatorHAL();
    ~MockHardwareEmulatorHAL() override;

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

    // Direct test inspection helpers
    void injectLimitFault(uint32_t axisId, bool maxLimit);
    void resetFaults(uint32_t axisId);

private:
    void physicsLoop();

    std::array<SimulatedAxisHardware, MAX_AXES> axes_{};
    mutable std::mutex hwMutex_;
    std::atomic<bool> running_{false};
    std::atomic<bool> estopActive_{false};
    std::thread physicsThread_;
    uint64_t tickCount_{0};
};

} // namespace MotorHub

#endif // MOCK_HARDWARE_EMULATOR_HAL_HPP

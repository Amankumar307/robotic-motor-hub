/**
 * @file MotorAxis.hpp
 * @brief Individual Actuator Motor Axis Controller with State Machine and Closed-Loop Regulation
 */

#ifndef MOTOR_AXIS_HPP
#define MOTOR_AXIS_HPP

#include "common/MotorTypes.hpp"
#include "hal/IHardwareDevice.hpp"
#include "core/PIDController.hpp"
#include "core/TrajectoryPlanner.hpp"

namespace MotorHub {

class MotorAxis {
public:
    MotorAxis(const AxisConfig& config, IHardwareDevice& hal);

    void update(double dt);

    bool moveTo(int32_t targetPosition);
    bool setVelocity(int32_t targetVelocity);
    bool home();
    void emergencyStop();
    bool resetFault();
    void setEnabled(bool enable);

    AxisState getState() const { return state_; }
    uint32_t getAxisId() const { return config_.axisId; }
    const AxisConfig& getConfig() const { return config_; }
    AxisTelemetry getTelemetry() const;

    void setPIDParameters(const PIDParameters& params);
    void setSynchronizedProfile(const TrapezoidalProfile& profile);

private:
    void transitionTo(AxisState newState);

    AxisConfig config_;
    IHardwareDevice& hal_;
    AxisState state_{AxisState::UNINITIALIZED};
    PIDController pid_;

    TrapezoidalProfile activeProfile_{};
    double trajectoryElapsedSec_{0.0};
    bool trajectoryActive_{false};

    int32_t targetPosition_{0};
    int32_t targetVelocity_{0};
    int32_t actualPosition_{0};
    int32_t actualVelocity_{0};
    int16_t lastPwmEffort_{0};
    uint16_t statusFlags_{0};
    uint32_t temperatureC_{35};

    bool isHomed_{false};
};

} // namespace MotorHub

#endif // MOTOR_AXIS_HPP

/**
 * @file MotorTypes.hpp
 * @brief Common Types, Enums, and Telemetry Structures for Motor Hub
 */

#ifndef MOTOR_TYPES_HPP
#define MOTOR_TYPES_HPP

#include <cstdint>
#include <string>
#include <array>

namespace MotorHub {

constexpr size_t MAX_AXES = 6;

enum class AxisState : uint8_t {
    UNINITIALIZED   = 0,
    IDLE            = 1,
    HOMING          = 2,
    POSITIONING     = 3,
    JOGGING         = 4,
    ERROR           = 5,
    EMERGENCY_STOP  = 6
};

inline const char* toString(AxisState state) {
    switch (state) {
        case AxisState::UNINITIALIZED:  return "UNINITIALIZED";
        case AxisState::IDLE:           return "IDLE";
        case AxisState::HOMING:         return "HOMING";
        case AxisState::POSITIONING:    return "POSITIONING";
        case AxisState::JOGGING:        return "JOGGING";
        case AxisState::ERROR:          return "ERROR";
        case AxisState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        default:                        return "UNKNOWN";
    }
}

enum class CommandType : uint16_t {
    NOP             = 0,
    ENABLE_AXIS     = 1,
    DISABLE_AXIS    = 2,
    HOME_AXIS       = 3,
    MOVE_AXIS       = 4,
    SYNC_MOVE       = 5,
    SET_PID         = 6,
    RESET_FAULT     = 7,
    EMERGENCY_STOP  = 8,
    CLEAR_ESTOP     = 9,
    GET_STATUS      = 10
};

struct PIDParameters {
    double kp{1.2};
    double ki{0.05};
    double kd{0.08};
    double outputMin{-1000.0};
    double outputMax{1000.0};
    double integralLimit{300.0};
};

struct AxisConfig {
    uint32_t axisId{0};
    std::string name{"Axis_0"};
    int32_t minLimitTicks{-500000};
    int32_t maxLimitTicks{500000};
    int32_t maxVelocityTicksPerSec{100000};
    int32_t maxAccelerationTicksPerSec2{200000};
    int32_t positionDeadbandTicks{5};
    PIDParameters pid{};
};

struct alignas(64) AxisTelemetry {
    uint32_t  axisId{0};
    AxisState state{AxisState::UNINITIALIZED};
    int32_t   targetPosition{0};
    int32_t   actualPosition{0};
    int32_t   trackingError{0};
    int32_t   targetVelocity{0};
    int32_t   actualVelocity{0};
    int16_t   pwmEffort{0};
    uint16_t  statusFlags{0};
    uint32_t  temperatureC{35};
    uint64_t  timestampNs{0};
};

struct alignas(128) SystemTelemetryPacket {
    uint64_t sequenceNumber{0};
    uint64_t timestampNs{0};
    uint32_t activeAxes{MAX_AXES};
    bool     emergencyStopActive{false};
    std::array<AxisTelemetry, MAX_AXES> axes{};
};

struct CommandPacket {
    CommandType type{CommandType::NOP};
    uint32_t    axisId{0};
    int32_t     param1{0};
    int32_t     param2{0};
    int32_t     param3{0};
    std::array<int32_t, MAX_AXES> syncTargets{};
};

struct TrajectoryWaypoint {
    double timeSec{0.0};
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
};

} // namespace MotorHub

#endif // MOTOR_TYPES_HPP

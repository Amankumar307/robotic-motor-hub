/**
 * @file MultiAxisController.hpp
 * @brief Synchronized Multi-Axis Motion Controller and Coordinator
 */

#ifndef MULTI_AXIS_CONTROLLER_HPP
#define MULTI_AXIS_CONTROLLER_HPP

#include "core/MotorAxis.hpp"
#include "hal/IHardwareDevice.hpp"
#include <vector>
#include <memory>
#include <mutex>

namespace MotorHub {

class MultiAxisController {
public:
    explicit MultiAxisController(IHardwareDevice& hal);

    bool initializeAxes(const std::vector<AxisConfig>& configs);
    void updateControlLoop(double dt);

    bool moveSingleAxis(uint32_t axisId, int32_t targetPosition);
    bool moveSynchronized(const std::array<int32_t, MAX_AXES>& targetPositions);
    bool homeAxis(uint32_t axisId);
    bool homeAllAxes();

    void globalEmergencyStop();
    void clearEmergencyStop();
    bool resetAxisFault(uint32_t axisId);

    SystemTelemetryPacket getSystemTelemetry();
    MotorAxis* getAxis(uint32_t axisId);
    size_t getAxisCount() const { return axes_.size(); }

private:
    IHardwareDevice& hal_;
    std::vector<std::unique_ptr<MotorAxis>> axes_;
    mutable std::mutex controllerMutex_;
    uint64_t loopCounter_{0};
    bool globalEstopActive_{false};
};

} // namespace MotorHub

#endif // MULTI_AXIS_CONTROLLER_HPP

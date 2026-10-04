/**
 * @file TrajectoryPlanner.hpp
 * @brief Trapezoidal Velocity Profile and Multi-Axis Synchronized Trajectory Generator
 */

#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

#include "common/MotorTypes.hpp"
#include <vector>
#include <array>

namespace MotorHub {

struct TrapezoidalProfile {
    double startPos{0.0};
    double targetPos{0.0};
    double direction{1.0};
    double distance{0.0};
    double maxVel{0.0};
    double maxAccel{0.0};
    double tAccel{0.0};
    double tCruise{0.0};
    double tDecel{0.0};
    double totalDuration{0.0};
    double dAccel{0.0};
    double dCruise{0.0};
    bool   isFinished{true};
};

class TrajectoryPlanner {
public:
    TrajectoryPlanner() = default;

    static TrapezoidalProfile generateProfile(double startPos, double targetPos, double maxVel, double maxAccel);
    static TrajectoryWaypoint sampleProfile(const TrapezoidalProfile& profile, double elapsedSec);

    // Multi-Axis Synchronized Trajectory Generation
    static std::array<TrapezoidalProfile, MAX_AXES> generateSynchronizedProfiles(
        const std::array<double, MAX_AXES>& startPositions,
        const std::array<double, MAX_AXES>& targetPositions,
        const std::array<double, MAX_AXES>& maxVelocities,
        const std::array<double, MAX_AXES>& maxAccelerations);
};

} // namespace MotorHub

#endif // TRAJECTORY_PLANNER_HPP

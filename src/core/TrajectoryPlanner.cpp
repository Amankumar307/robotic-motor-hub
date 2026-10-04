/**
 * @file TrajectoryPlanner.cpp
 * @brief Trapezoidal Velocity Profile and Synchronized Trajectory Implementation
 */

#include "core/TrajectoryPlanner.hpp"
#include <cmath>
#include <algorithm>

namespace MotorHub {

TrapezoidalProfile TrajectoryPlanner::generateProfile(double startPos, double targetPos, double maxVel, double maxAccel) {
    TrapezoidalProfile prof;
    prof.startPos = startPos;
    prof.targetPos = targetPos;
    prof.distance = std::abs(targetPos - startPos);

    if (prof.distance < 1e-4 || maxVel <= 1e-4 || maxAccel <= 1e-4) {
        prof.isFinished = true;
        prof.totalDuration = 0.0;
        return prof;
    }

    prof.direction = (targetPos >= startPos) ? 1.0 : -1.0;
    prof.maxVel = std::abs(maxVel);
    prof.maxAccel = std::abs(maxAccel);
    prof.isFinished = false;

    // Distance required to reach max velocity: v^2 / a
    double distToMaxVel = (prof.maxVel * prof.maxVel) / prof.maxAccel;

    if (distToMaxVel > prof.distance) {
        // Triangular profile (does not reach full maxVel)
        double peakVel = std::sqrt(prof.distance * prof.maxAccel);
        prof.tAccel = peakVel / prof.maxAccel;
        prof.tCruise = 0.0;
        prof.tDecel = prof.tAccel;
        prof.totalDuration = 2.0 * prof.tAccel;
        prof.dAccel = 0.5 * prof.distance;
        prof.dCruise = 0.0;
    } else {
        // Full trapezoidal profile
        prof.tAccel = prof.maxVel / prof.maxAccel;
        prof.dAccel = 0.5 * prof.maxAccel * prof.tAccel * prof.tAccel;
        prof.dCruise = prof.distance - (2.0 * prof.dAccel);
        prof.tCruise = prof.dCruise / prof.maxVel;
        prof.tDecel = prof.tAccel;
        prof.totalDuration = prof.tAccel + prof.tCruise + prof.tDecel;
    }

    return prof;
}

TrajectoryWaypoint TrajectoryPlanner::sampleProfile(const TrapezoidalProfile& profile, double elapsedSec) {
    TrajectoryWaypoint pt;
    pt.timeSec = elapsedSec;

    if (profile.isFinished || elapsedSec <= 0.0) {
        pt.position = profile.startPos;
        pt.velocity = 0.0;
        pt.acceleration = 0.0;
        return pt;
    }

    if (elapsedSec >= profile.totalDuration) {
        pt.position = profile.targetPos;
        pt.velocity = 0.0;
        pt.acceleration = 0.0;
        return pt;
    }

    double t = elapsedSec;
    double tA = profile.tAccel;
    double tC = profile.tCruise;
    double a = profile.maxAccel;
    double vMax = (tC > 0.0) ? profile.maxVel : (a * tA);

    if (t <= tA) {
        // Phase 1: Acceleration
        double d = 0.5 * a * t * t;
        pt.position = profile.startPos + profile.direction * d;
        pt.velocity = profile.direction * (a * t);
        pt.acceleration = profile.direction * a;
    } else if (t <= tA + tC) {
        // Phase 2: Constant Velocity Cruise
        double d = profile.dAccel + vMax * (t - tA);
        pt.position = profile.startPos + profile.direction * d;
        pt.velocity = profile.direction * vMax;
        pt.acceleration = 0.0;
    } else {
        // Phase 3: Deceleration
        double tDec = t - (tA + tC);
        double d = profile.dAccel + profile.dCruise + (vMax * tDec - 0.5 * a * tDec * tDec);
        pt.position = profile.startPos + profile.direction * d;
        pt.velocity = profile.direction * (vMax - a * tDec);
        pt.acceleration = -profile.direction * a;
    }

    return pt;
}

std::array<TrapezoidalProfile, MAX_AXES> TrajectoryPlanner::generateSynchronizedProfiles(
    const std::array<double, MAX_AXES>& startPositions,
    const std::array<double, MAX_AXES>& targetPositions,
    const std::array<double, MAX_AXES>& maxVelocities,
    const std::array<double, MAX_AXES>& maxAccelerations)
{
    std::array<TrapezoidalProfile, MAX_AXES> profiles{};
    double maxDuration = 0.0;

    // Step 1: Calculate standalone profiles to find maximum required time
    for (size_t i = 0; i < MAX_AXES; ++i) {
        profiles[i] = generateProfile(startPositions[i], targetPositions[i], maxVelocities[i], maxAccelerations[i]);
        if (!profiles[i].isFinished && profiles[i].totalDuration > maxDuration) {
            maxDuration = profiles[i].totalDuration;
        }
    }

    if (maxDuration <= 1e-4) {
        return profiles;
    }

    // Step 2: Scale each axis velocity and acceleration so all axes finish at maxDuration
    for (size_t i = 0; i < MAX_AXES; ++i) {
        if (profiles[i].distance <= 1e-4) continue;

        double d = profiles[i].distance;
        // Set cruise time proportion based on original profile
        double scale = profiles[i].totalDuration / maxDuration;
        double newVel = profiles[i].maxVel * scale;
        double newAccel = profiles[i].maxAccel * (scale * scale);

        // Re-generate scaled profile
        profiles[i] = generateProfile(startPositions[i], targetPositions[i], newVel, newAccel);
    }

    return profiles;
}

} // namespace MotorHub

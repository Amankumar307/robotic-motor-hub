/**
 * @file PIDController.cpp
 * @brief Discrete PID Controller Implementation
 */

#include "core/PIDController.hpp"
#include <algorithm>
#include <cmath>

namespace MotorHub {

PIDController::PIDController(const PIDParameters& params)
    : params_(params) {}

void PIDController::setParameters(const PIDParameters& params) {
    params_ = params;
    reset();
}

void PIDController::reset() {
    integral_ = 0.0;
    prevError_ = 0.0;
    filteredDerivative_ = 0.0;
    firstRun_ = true;
}

double PIDController::compute(double target, double actual, double dt) {
    if (dt <= 1e-6) return 0.0;

    double error = target - actual;

    // Proportional term
    double pTerm = params_.kp * error;

    // Integral term with anti-windup clamping
    integral_ += error * dt;
    if (params_.integralLimit > 0.0) {
        integral_ = std::clamp(integral_, -params_.integralLimit, params_.integralLimit);
    }
    double iTerm = params_.ki * integral_;

    // Derivative term with low-pass filter (cutoff ~ 100 Hz)
    double derivative = 0.0;
    if (!firstRun_) {
        derivative = (error - prevError_) / dt;
    } else {
        firstRun_ = false;
    }
    prevError_ = error;

    constexpr double filterAlpha = 0.7; // Filter factor
    filteredDerivative_ = filterAlpha * derivative + (1.0 - filterAlpha) * filteredDerivative_;
    double dTerm = params_.kd * filteredDerivative_;

    double output = pTerm + iTerm + dTerm;
    return std::clamp(output, params_.outputMin, params_.outputMax);
}

} // namespace MotorHub

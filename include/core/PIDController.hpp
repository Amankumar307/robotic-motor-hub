/**
 * @file PIDController.hpp
 * @brief Discrete-time PID Controller with Anti-Windup Clamping and Filtered Derivative
 */

#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include "common/MotorTypes.hpp"

namespace MotorHub {

class PIDController {
public:
    PIDController() = default;
    explicit PIDController(const PIDParameters& params);

    void setParameters(const PIDParameters& params);
    const PIDParameters& getParameters() const { return params_; }

    double compute(double target, double actual, double dt);
    void reset();

private:
    PIDParameters params_{};
    double integral_{0.0};
    double prevError_{0.0};
    double filteredDerivative_{0.0};
    bool firstRun_{true};
};

} // namespace MotorHub

#endif // PID_CONTROLLER_HPP

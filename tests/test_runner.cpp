/**
 * @file test_runner.cpp
 * @brief Comprehensive Automated Test Suite for Robotic Motor Controller Hub
 */

#include "common/MotorTypes.hpp"
#include "common/HardwareRegisters.hpp"
#include "hal/MockHardwareEmulatorHAL.hpp"
#include "core/PIDController.hpp"
#include "core/TrajectoryPlanner.hpp"
#include "core/MotorAxis.hpp"
#include "core/MultiAxisController.hpp"
#include "core/FailsafeWatchdog.hpp"

#include <iostream>
#include <cassert>
#include <cmath>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

namespace {
int g_passedTests = 0;
int g_failedTests = 0;

void reportTest(const std::string& name, bool passed) {
    if (passed) {
        std::cout << "  [PASS] " << name << std::endl;
        g_passedTests++;
    } else {
        std::cout << "  [FAIL] " << name << std::endl;
        g_failedTests++;
    }
}
} // namespace

void testPIDController() {
    std::cout << "\n=== 1. Testing Discrete-Time PID Controller ===" << std::endl;

    MotorHub::PIDParameters params{};
    params.kp = 2.0;
    params.ki = 0.5;
    params.kd = 0.1;
    params.outputMin = -1000.0;
    params.outputMax = 1000.0;
    params.integralLimit = 100.0;

    MotorHub::PIDController pid(params);

    // Test 1: Zero error yields zero output on first run
    double out1 = pid.compute(0.0, 0.0, 0.001);
    reportTest("Zero error yields zero output", std::abs(out1) < 1e-4);

    // Test 2: Step response generates positive output proportional to error
    double outStep = pid.compute(100.0, 0.0, 0.001);
    reportTest("Positive error generates positive control effort", outStep > 0.0);

    // Test 3: Anti-windup clamping prevents integral saturation
    for (int i = 0; i < 5000; ++i) {
        pid.compute(1000.0, 0.0, 0.001);
    }
    double clampedOut = pid.compute(1000.0, 0.0, 0.001);
    reportTest("Output respects upper limit (1000.0)", clampedOut <= 1000.0);

    // Test 4: Reset clears state
    pid.reset();
    double outAfterReset = pid.compute(0.0, 0.0, 0.001);
    reportTest("PID reset clears integral and derivative history", std::abs(outAfterReset) < 1e-4);
}

void testTrajectoryPlanner() {
    std::cout << "\n=== 2. Testing Trajectory Planner (Trapezoidal Profiles) ===" << std::endl;

    // Test 1: Profile duration calculation
    double start = 0.0;
    double target = 50000.0;
    double maxVel = 50000.0;
    double maxAccel = 100000.0;

    auto prof = MotorHub::TrajectoryPlanner::generateProfile(start, target, maxVel, maxAccel);
    reportTest("Profile distance calculation is exact", std::abs(prof.distance - 50000.0) < 1e-4);
    reportTest("Profile duration is positive", prof.totalDuration > 0.0);

    // Test 2: Sample at t = 0 matches start position
    auto sample0 = MotorHub::TrajectoryPlanner::sampleProfile(prof, 0.0);
    reportTest("Sample at t=0 matches start position", std::abs(sample0.position - start) < 1e-4);

    // Test 3: Sample at t >= totalDuration matches target position
    auto sampleEnd = MotorHub::TrajectoryPlanner::sampleProfile(prof, prof.totalDuration + 0.1);
    reportTest("Sample at completion reaches exact target position", std::abs(sampleEnd.position - target) < 1e-4);

    // Test 4: Synchronized multi-axis profiles
    std::array<double, MotorHub::MAX_AXES> startPos{0, 0, 0, 0, 0, 0};
    std::array<double, MotorHub::MAX_AXES> targetPos{10000, 50000, 25000, 0, 0, 0};
    std::array<double, MotorHub::MAX_AXES> maxVels{50000, 50000, 50000, 50000, 50000, 50000};
    std::array<double, MotorHub::MAX_AXES> maxAccels{100000, 100000, 100000, 100000, 100000, 100000};

    auto syncProfiles = MotorHub::TrajectoryPlanner::generateSynchronizedProfiles(startPos, targetPos, maxVels, maxAccels);
    double durationX = syncProfiles[0].totalDuration;
    double durationY = syncProfiles[1].totalDuration;
    double durationZ = syncProfiles[2].totalDuration;

    reportTest("Synchronized profiles all complete within identical timeframe",
               std::abs(durationX - durationY) < 1e-3 && std::abs(durationY - durationZ) < 1e-3);
}

void testHardwareEmulatorAndHAL() {
    std::cout << "\n=== 3. Testing Mock Hardware Emulator & HAL ===" << std::endl;

    MotorHub::MockHardwareEmulatorHAL hal;
    bool initOk = hal.initialize();
    reportTest("Hardware Emulator initializes successfully", initOk);

    // Test 1: Register read/write roundtrip
    hal.writeRegister(0, MotorHub::REG_OFFSET_TARGET_POS, 12345);
    uint32_t val = hal.readRegister(0, MotorHub::REG_OFFSET_TARGET_POS);
    reportTest("Register read/write roundtrip matches (target_pos)", val == 12345);

    // Test 2: Status register default enabled
    uint32_t status = hal.readRegister(0, MotorHub::REG_OFFSET_STATUS);
    reportTest("Axis 0 status indicates ENABLED", (status & MotorHub::StatusBits::ENABLED) != 0);

    // Test 3: Emergency stop clears PWM and latches ESTOP bit
    hal.triggerEmergencyStop();
    reportTest("HAL reports emergency stop active", hal.isEmergencyStopActive());

    uint32_t estopStatus = hal.readRegister(0, MotorHub::REG_OFFSET_STATUS);
    reportTest("Hardware status register asserts ESTOP_ACTIVE bit",
               (estopStatus & MotorHub::StatusBits::ESTOP_ACTIVE) != 0);

    hal.clearEmergencyStop();
    reportTest("Emergency stop clears successfully", !hal.isEmergencyStopActive());

    hal.shutdown();
}

void testMultiAxisControllerAndWatchdog() {
    std::cout << "\n=== 4. Testing Multi-Axis Motion & Safety Watchdog ===" << std::endl;

    MotorHub::MockHardwareEmulatorHAL hal;
    hal.initialize();

    std::vector<MotorHub::AxisConfig> configs;
    for (uint32_t i = 0; i < 3; ++i) {
        MotorHub::AxisConfig cfg{};
        cfg.axisId = i;
        cfg.name = "Axis_" + std::to_string(i);
        cfg.minLimitTicks = -200000;
        cfg.maxLimitTicks = 200000;
        cfg.maxVelocityTicksPerSec = 50000;
        cfg.maxAccelerationTicksPerSec2 = 100000;
        cfg.positionDeadbandTicks = 10;
        configs.push_back(cfg);
    }

    MotorHub::MultiAxisController controller(hal);
    controller.initializeAxes(configs);
    reportTest("MultiAxisController initialized with 3 axes", controller.getAxisCount() == 3);

    // Move single axis
    bool moveOk = controller.moveSingleAxis(0, 5000);
    reportTest("Axis 0 move command dispatched", moveOk);

    // Step control loop for 200 iterations (200 ms)
    for (int i = 0; i < 200; ++i) {
        controller.updateControlLoop(0.001);
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    auto telem = controller.getSystemTelemetry();
    reportTest("Axis 0 actual position moved towards target", telem.axes[0].actualPosition != 0);

    // Test Watchdog Heartbeat Supervision
    MotorHub::FailsafeWatchdog watchdog(controller, 50000, 80, std::chrono::milliseconds(50));
    watchdog.start();
    watchdog.kick();

    // Sleep longer than timeout without kicking to simulate hung loop
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    reportTest("Watchdog tripped on missed heartbeat deadline", watchdog.isTripped());
    reportTest("Watchdog forced global emergency stop", hal.isEmergencyStopActive());

    watchdog.stop();
    hal.shutdown();
}

int main() {
    std::cout << "=========================================================\n"
              << "   Robotic Motor Controller Hub - Test & Verification    \n"
              << "=========================================================\n";

    auto start = std::chrono::steady_clock::now();

    testPIDController();
    testTrajectoryPlanner();
    testHardwareEmulatorAndHAL();
    testMultiAxisControllerAndWatchdog();

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();

    std::cout << "\n---------------------------------------------------------\n"
              << "Test Execution Summary:\n"
              << "  Total Tests Executed: " << (g_passedTests + g_failedTests) << "\n"
              << "  Passed:               " << g_passedTests << "\n"
              << "  Failed:               " << g_failedTests << "\n"
              << "  Elapsed Time:         " << elapsed << " ms\n"
              << "---------------------------------------------------------\n";

    if (g_failedTests == 0) {
        std::cout << "ALL TEST CASES PASSED SUCCESSFULLY [100% SUCCESS]\n";
        return 0;
    } else {
        std::cerr << "SOME TESTS FAILED!\n";
        return 1;
    }
}

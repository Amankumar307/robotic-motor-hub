/**
 * @file main_controller_daemon.cpp
 * @brief High-Precision 1 kHz Real-Time Daemon for Robotic Motor Controller Hub
 * @author Robotics & Embedded Systems Engineering
 */

#include "common/MotorTypes.hpp"
#include "common/Logger.hpp"
#include "hal/KernelDeviceDriverHAL.hpp"
#include "hal/MockHardwareEmulatorHAL.hpp"
#include "core/MultiAxisController.hpp"
#include "core/FailsafeWatchdog.hpp"
#include "ipc/SharedMemoryTelemetry.hpp"
#include "ipc/MessageQueueCommandServer.hpp"

#include <iostream>
#include <memory>
#include <vector>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>
#include <cstring>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

namespace {
std::atomic<bool> g_running{true};
std::atomic<bool> g_sigintReceived{false};

void signalHandler(int signum) {
    if (signum == SIGINT || signum == SIGTERM) {
        g_sigintReceived = true;
        g_running = false;
    }
}
} // namespace

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    LOG_INFO("Daemon", "=======================================================");
    LOG_INFO("Daemon", "Starting Robotic Multi-Axis Motor Controller Hub Daemon");
    LOG_INFO("Daemon", "Domain 5: Robotics, Edge AI & Hardware Emulation");
    LOG_INFO("Daemon", "=======================================================");

    bool forceEmulation = false;
    std::string devicePath = "/dev/motor_hub";

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--emulation") == 0 || std::strcmp(argv[i], "-e") == 0) {
            forceEmulation = true;
        } else if (std::strcmp(argv[i], "--device") == 0 && i + 1 < argc) {
            devicePath = argv[++i];
        }
    }

#if defined(__linux__)
    // Attempt to set POSIX Real-Time FIFO scheduling
    struct sched_param param{};
    param.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0) {
        LOG_INFO("Daemon", "Real-Time SCHED_FIFO priority (80) successfully acquired.");
    } else {
        LOG_WARN("Daemon", "Could not set SCHED_FIFO (run with sudo/CAP_SYS_NICE for hard RT priority).");
    }
#endif

    // Select Hardware Abstraction Layer
    std::unique_ptr<MotorHub::IHardwareDevice> hal;
    if (!forceEmulation) {
        auto kernelHal = std::make_unique<MotorHub::KernelDeviceDriverHAL>(devicePath);
        if (kernelHal->initialize()) {
            LOG_INFO("Daemon", "Connected to Linux Kernel Device Driver at " + devicePath);
            hal = std::move(kernelHal);
        } else {
            LOG_WARN("Daemon", "Kernel driver unavailable. Falling back to Mock Hardware Emulator.");
            forceEmulation = true;
        }
    }

    if (forceEmulation) {
        auto mockHal = std::make_unique<MotorHub::MockHardwareEmulatorHAL>();
        mockHal->initialize();
        LOG_INFO("Daemon", "Running in High-Fidelity Mock Hardware Emulation Mode.");
        hal = std::move(mockHal);
    }

    // Initialize 6 Robot Axes (X, Y, Z, Roll, Pitch, Yaw)
    std::vector<MotorHub::AxisConfig> configs;
    const std::vector<std::string> axisNames = {"Axis_X", "Axis_Y", "Axis_Z", "Axis_Roll", "Axis_Pitch", "Axis_Yaw"};
    for (uint32_t i = 0; i < MotorHub::MAX_AXES; ++i) {
        MotorHub::AxisConfig cfg{};
        cfg.axisId = i;
        cfg.name = axisNames[i];
        cfg.minLimitTicks = -500000;
        cfg.maxLimitTicks = 500000;
        cfg.maxVelocityTicksPerSec = 80000;
        cfg.maxAccelerationTicksPerSec2 = 160000;
        cfg.positionDeadbandTicks = 10;
        cfg.pid.kp = 1.4;
        cfg.pid.ki = 0.08;
        cfg.pid.kd = 0.05;
        cfg.pid.outputMin = -1000.0;
        cfg.pid.outputMax = 1000.0;
        cfg.pid.integralLimit = 250.0;
        configs.push_back(cfg);
    }

    MotorHub::MultiAxisController controller(*hal);
    controller.initializeAxes(configs);

    // Initialize Watchdog & IPC Telemetry Publisher
    MotorHub::FailsafeWatchdog watchdog(controller, 35000, 85, std::chrono::milliseconds(150));
    watchdog.start();

    MotorHub::SharedMemoryPublisher shmPublisher("/motor_hub_telemetry");
    if (!shmPublisher.initialize()) {
        LOG_WARN("Daemon", "Continuing without Shared Memory Publisher.");
    }

    LOG_INFO("Daemon", "Entering Deterministic 1 kHz (1.0 ms) Periodic Control Loop...");

    using clock = std::chrono::steady_clock;
    constexpr auto loopPeriod = std::chrono::microseconds(1000); // 1 ms = 1000 us
    auto nextCycle = clock::now();
    uint64_t cycleCount = 0;
    auto lastTelemetryLog = clock::now();

    while (g_running) {
        nextCycle += loopPeriod;
        auto cycleStart = clock::now();

        // 1. Process asynchronous IPC operator commands
        MotorHub::CommandPacket cmd;
        while (MotorHub::CommandQueueServer::getInstance().popCommand(cmd, 0)) {
            switch (cmd.type) {
                case MotorHub::CommandType::MOVE_AXIS:
                    controller.moveSingleAxis(cmd.axisId, cmd.param1);
                    break;
                case MotorHub::CommandType::SYNC_MOVE:
                    controller.moveSynchronized(cmd.syncTargets);
                    break;
                case MotorHub::CommandType::HOME_AXIS:
                    controller.homeAxis(cmd.axisId);
                    break;
                case MotorHub::CommandType::EMERGENCY_STOP:
                    controller.globalEmergencyStop();
                    break;
                case MotorHub::CommandType::CLEAR_ESTOP:
                    controller.clearEmergencyStop();
                    watchdog.resetTrip();
                    break;
                case MotorHub::CommandType::RESET_FAULT:
                    controller.resetAxisFault(cmd.axisId);
                    break;
                default:
                    break;
            }
        }

        // 2. Execute 1 kHz Multi-Axis Closed-Loop Regulation Step
        controller.updateControlLoop(0.001);

        // 3. Heartbeat kick to safety watchdog
        watchdog.kick();

        // 4. Publish zero-copy telemetry packet to Shared Memory
        auto telemetryPacket = controller.getSystemTelemetry();
        shmPublisher.publish(telemetryPacket);

        cycleCount++;

        // Periodic diagnostic log every 2 seconds
        if (clock::now() - lastTelemetryLog > std::chrono::seconds(2)) {
            lastTelemetryLog = clock::now();
            std::string statusStr = "Cycle: " + std::to_string(cycleCount) + " | Pos [";
            for (size_t i = 0; i < 3; ++i) {
                statusStr += configs[i].name + ":" + std::to_string(telemetryPacket.axes[i].actualPosition) + " ";
            }
            statusStr += "] E-Stop: " + std::string(telemetryPacket.emergencyStopActive ? "YES" : "NO");
            LOG_INFO("Daemon", statusStr);
        }

        // Sleep until next deterministic 1.0 ms boundary
        std::this_thread::sleep_until(nextCycle);
    }

    LOG_WARN("Daemon", "Shutdown signal received. Initiating graceful shutdown...");
    watchdog.stop();
    controller.globalEmergencyStop();
    shmPublisher.close();
    hal->shutdown();

    LOG_INFO("Daemon", "Robotic Motor Controller Hub Daemon terminated safely.");
    return 0;
}

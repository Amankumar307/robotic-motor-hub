/**
 * @file motor_cli_client.cpp
 * @brief Interactive Operator Command-Line Interface (CLI) Client
 */

#include "common/MotorTypes.hpp"
#include "common/Logger.hpp"
#include "ipc/SharedMemoryTelemetry.hpp"
#include "ipc/MessageQueueCommandServer.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <thread>

void printHelp() {
    std::cout << "\n=======================================================\n"
              << "       Robotic Motor Controller Hub CLI Commands       \n"
              << "=======================================================\n"
              << "  status                 - Display current telemetry table\n"
              << "  move <axis> <ticks>    - Move single axis to position\n"
              << "  sync <x> <y> <z>       - Coordinated multi-axis move\n"
              << "  home <axis>            - Home axis to reference zero\n"
              << "  estop                  - Trigger Emergency Stop\n"
              << "  clear                  - Clear E-Stop & reset faults\n"
              << "  monitor                - Live 10 Hz telemetry dashboard\n"
              << "  help                   - Display this command list\n"
              << "  exit                   - Exit CLI client\n"
              << "=======================================================\n" << std::endl;
}

void printTelemetryTable(const MotorHub::SystemTelemetryPacket& telem) {
    std::cout << "\n+------+-------------+-----------+-----------+---------+------+-------+------+\n"
              << "| Axis |    State    | TargetPos | ActualPos | Error   | Vel  | PWM   | Temp |\n"
              << "+------+-------------+-----------+-----------+---------+------+-------+------+\n";

    for (size_t i = 0; i < telem.activeAxes; ++i) {
        const auto& ax = telem.axes[i];
        std::cout << "|  " << std::setw(2) << ax.axisId << "  | "
                  << std::setw(11) << MotorHub::toString(ax.state) << " | "
                  << std::setw(9) << ax.targetPosition << " | "
                  << std::setw(9) << ax.actualPosition << " | "
                  << std::setw(7) << ax.trackingError << " | "
                  << std::setw(4) << ax.actualVelocity << " | "
                  << std::setw(5) << ax.pwmEffort << " | "
                  << std::setw(4) << ax.temperatureC << "C |\n";
    }
    std::cout << "+------+-------------+-----------+-----------+---------+------+-------+------+\n";
    std::cout << "E-Stop Active: " << (telem.emergencyStopActive ? "[YES - TRIPPED]" : "[NO - NORMAL]")
              << " | Seq: " << telem.sequenceNumber << "\n" << std::endl;
}

int main() {
    std::cout << "Robotic Multi-Axis Motor Controller & Actuator Hub CLI v1.0.0\n";
    std::cout << "Type 'help' for available commands.\n";

    MotorHub::SharedMemorySubscriber shmSub("/motor_hub_telemetry");
    if (!shmSub.initialize()) {
        std::cout << "[Note] Shared Memory not active. Commands will queue locally.\n";
    }

    std::string line;
    while (true) {
        std::cout << "motor_hub> ";
        if (!std::getline(std::cin, line)) break;

        std::stringstream ss(line);
        std::string cmd;
        ss >> cmd;

        if (cmd.empty()) continue;

        if (cmd == "exit" || cmd == "quit") {
            break;
        } else if (cmd == "help") {
            printHelp();
        } else if (cmd == "status") {
            MotorHub::SystemTelemetryPacket telem{};
            if (shmSub.readLatest(telem)) {
                printTelemetryTable(telem);
            } else {
                std::cout << "[WARN] Unable to read telemetry from Shared Memory.\n";
            }
        } else if (cmd == "estop") {
            MotorHub::CommandPacket packet{};
            packet.type = MotorHub::CommandType::EMERGENCY_STOP;
            MotorHub::CommandQueueServer::getInstance().pushCommand(packet);
            std::cout << "[CRITICAL] Emergency Stop command queued!\n";
        } else if (cmd == "clear") {
            MotorHub::CommandPacket packet{};
            packet.type = MotorHub::CommandType::CLEAR_ESTOP;
            MotorHub::CommandQueueServer::getInstance().pushCommand(packet);
            std::cout << "[INFO] Clear E-Stop command queued.\n";
        } else if (cmd == "move") {
            uint32_t axis = 0;
            int32_t target = 0;
            if (ss >> axis >> target) {
                MotorHub::CommandPacket packet{};
                packet.type = MotorHub::CommandType::MOVE_AXIS;
                packet.axisId = axis;
                packet.param1 = target;
                MotorHub::CommandQueueServer::getInstance().pushCommand(packet);
                std::cout << "[INFO] Move command queued: Axis " << axis << " -> " << target << " ticks.\n";
            } else {
                std::cout << "Usage: move <axis_id> <target_position_ticks>\n";
            }
        } else if (cmd == "sync") {
            int32_t x = 0, y = 0, z = 0;
            if (ss >> x >> y >> z) {
                MotorHub::CommandPacket packet{};
                packet.type = MotorHub::CommandType::SYNC_MOVE;
                packet.syncTargets[0] = x;
                packet.syncTargets[1] = y;
                packet.syncTargets[2] = z;
                MotorHub::CommandQueueServer::getInstance().pushCommand(packet);
                std::cout << "[INFO] Synchronized Move queued: X=" << x << ", Y=" << y << ", Z=" << z << "\n";
            } else {
                std::cout << "Usage: sync <x_ticks> <y_ticks> <z_ticks>\n";
            }
        } else if (cmd == "home") {
            uint32_t axis = 0;
            if (ss >> axis) {
                MotorHub::CommandPacket packet{};
                packet.type = MotorHub::CommandType::HOME_AXIS;
                packet.axisId = axis;
                MotorHub::CommandQueueServer::getInstance().pushCommand(packet);
                std::cout << "[INFO] Home command queued for Axis " << axis << ".\n";
            } else {
                std::cout << "Usage: home <axis_id>\n";
            }
        } else if (cmd == "monitor") {
            std::cout << "Streaming live telemetry (press Ctrl+C to stop)...\n";
            for (int i = 0; i < 20; ++i) {
                MotorHub::SystemTelemetryPacket telem{};
                if (shmSub.readLatest(telem)) {
                    printTelemetryTable(telem);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        } else {
            std::cout << "Unknown command: '" << cmd << "'. Type 'help' for instructions.\n";
        }
    }

    std::cout << "Exiting Motor Hub CLI.\n";
    return 0;
}

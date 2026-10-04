# Robotic Multi-Axis Motor Controller & Actuator Hub

[![Domain](https://img.shields.io/badge/Domain_5-Robotics_%26_Hardware_Emulation-blue.svg)](#)
[![Linux Kernel](https://img.shields.io/badge/Linux_Kernel-Character_Device_Driver-orange.svg)](#)
[![Language](https://img.shields.io/badge/Language-C%2B%2B17_%7C_C-green.svg)](#)
[![System Programming](https://img.shields.io/badge/System_Programming-POSIX_RT_%7C_IPC-red.svg)](#)
[![Status](https://img.shields.io/badge/Stages-1_to_6_Complete-brightgreen.svg)](#)

A high-performance, safety-critical motion control and actuator hub for multi-degree-of-freedom robotic systems (articulated robotic arms, CNC gantries, and automated guided vehicles). This project bridges user-space motion planning with kernel-space hardware registers, delivering a deterministic **1 kHz (1.0 ms)** closed-loop control cycle, hardware-level emergency stop watchdogs, and zero-copy shared memory telemetry.

Developed as a 6-Stage Capstone Project covering **Linux Device Drivers, System Programming, Computer Architecture, and Modern C++**.

---

## 📑 6-Stage Project Lifecycle & Documentation

| Stage | Title | Documentation Link | Deliverables |
|---|---|---|---|
| **Stage 1** | **Project Introduction** | [Stage 1 Document](file:///d:/Desktop/AIDOCBOT/robotic_motor_hub/docs/stage1_project_introduction.md) | Problem statement, objectives, robotics applications, and scope. |
| **Stage 2** | **Project Requirements & Development Plan** | [Stage 2 Document](file:///d:/Desktop/AIDOCBOT/robotic_motor_hub/docs/stage2_requirements_and_prd.md) | Full PRD (10 Functional & 8 Non-Functional requirements), Gantt timeline. |
| **Stage 3** | **System Design & Architecture** | [Stage 3 Document](file:///d:/Desktop/AIDOCBOT/robotic_motor_hub/docs/stage3_system_design_and_architecture.md) | System architecture, MMIO register layout, UML Class, Sequence, and State Machine diagrams. |
| **Stage 4** | **Initial Implementation & Prototype** | [Stage 4 Document](file:///d:/Desktop/AIDOCBOT/robotic_motor_hub/docs/stage4_prototype_implementation.md) | Character device driver, core HAL, initial prototype walkthrough, and challenges log. |
| **Stage 5** | **Testing, Integration & Improvement** | [Stage 5 Document](file:///d:/Desktop/AIDOCBOT/robotic_motor_hub/docs/stage5_testing_and_benchmarks.md) | Unit test suite, Pick-and-Place trajectory simulation, benchmarks, and latency profiling. |
| **Stage 6** | **Final Implementation & Presentation** | [Stage 6 Document](file:///d:/Desktop/AIDOCBOT/robotic_motor_hub/docs/stage6_final_report_and_presentation.md) | Final production report, slide-by-slide presentation deck script, and future roadmap. |

---

## 🏛️ System Architecture

```mermaid
graph TD
    subgraph UserLayer["User & Operator Layer"]
        CLI["Motor CLI Client (Interactive Terminal)"]
        Dashboard["Telemetry Dashboard / Monitor"]
    end

    subgraph IPCLayer["Inter-Process Communication (IPC)"]
        SHM["POSIX Shared Memory (/dev/shm/motor_hub_telemetry)"]
        MQ["Command Queue / IPC Channel"]
    end

    subgraph ControlCore["Real-Time Motion Control Core (C++ Daemon)"]
        Daemon["1 kHz Control Loop (SCHED_FIFO RT Priority)"]
        Coordinator["Multi-Axis Coordinator"]
        Planner["Trapezoidal Trajectory Profiler"]
        PID["Discrete Anti-Windup PID Regulator"]
        Watchdog["Safety Failsafe Watchdog"]
    end

    subgraph HALayer["Hardware Abstraction Layer (HAL)"]
        IHAL["<<interface>> IHardwareDevice"]
        KernelHAL["KernelDeviceDriverHAL (/dev/motor_hub)"]
        MockHAL["MockHardwareEmulatorHAL (Virtual MMIO + Physics)"]
    end

    subgraph KernelHW["Linux Kernel & Hardware"]
        Driver["motor_hub_driver.ko (Char Dev & 1 kHz hrtimer)"]
        MMIO["Hardware MMIO Registers (PWM, Encoders, IRQs)"]
    end

    CLI -->|Commands| MQ
    MQ --> Daemon
    Daemon --> Coordinator
    Coordinator --> Planner
    Coordinator --> PID
    Daemon --> Watchdog
    Coordinator --> IHAL
    IHAL <|.. KernelHAL
    IHAL <|.. MockHAL
    KernelHAL --> Driver
    MockHAL --> MMIO
    Driver --> MMIO
    Daemon -->|Publish Telemetry| SHM
    SHM --> CLI
    SHM --> Dashboard
```

---

## 📁 Repository Directory Structure

```
robotic_motor_hub/
├── CMakeLists.txt                      # Root CMake build configuration
├── Makefile                            # Top-level Makefile for driver, daemon, CLI, tests
├── README.md                           # Master project guide & architecture overview
├── docs/                               # Comprehensive 6-Stage Engineering Documentation
│   ├── stage1_project_introduction.md
│   ├── stage2_requirements_and_prd.md
│   ├── stage3_system_design_and_architecture.md
│   ├── stage4_prototype_implementation.md
│   ├── stage5_testing_and_benchmarks.md
│   └── stage6_final_report_and_presentation.md
├── driver/                             # Linux Kernel Module
│   ├── Kbuild                          # Kernel build specification
│   ├── Makefile                        # Kernel module build makefile
│   ├── motor_hub_driver.c              # Complete char driver (ioctl, sysfs, hrtimer)
│   ├── motor_hub_driver.h              # Kernel internal data structures
│   └── motor_hub_uapi.h                # User-kernel shared IOCTL & register offsets
├── include/                            # C++ Architecture Headers
│   ├── common/
│   │   ├── HardwareRegisters.hpp       # MMIO register definitions & bitmasks
│   │   ├── Logger.hpp                  # Thread-safe structured logging engine
│   │   └── MotorTypes.hpp              # Enums, configs, telemetry packets
│   ├── hal/
│   │   ├── IHardwareDevice.hpp         # Abstract hardware interface
│   │   ├── KernelDeviceDriverHAL.hpp   # Linux /dev/motor_hub backend
│   │   └── MockHardwareEmulatorHAL.hpp # High-fidelity virtual hardware emulator
│   ├── core/
│   │   ├── PIDController.hpp           # Discrete anti-windup PID regulator
│   │   ├── TrajectoryPlanner.hpp       # Trapezoidal multi-axis velocity profiler
│   │   ├── MotorAxis.hpp               # Axis controller & state machine
│   │   ├── MultiAxisController.hpp     # Coordinated multi-axis motion manager
│   │   └── FailsafeWatchdog.hpp        # Heartbeat & stall safety watchdog
│   └── ipc/
│       ├── SharedMemoryTelemetry.hpp   # Zero-copy shared memory publisher/subscriber
│       └── MessageQueueCommandServer.hpp# Asynchronous command queue
├── src/                                # C++ Implementation Units
│   ├── common/
│   ├── hal/
│   ├── core/
│   ├── ipc/
│   ├── daemon/
│   │   └── main_controller_daemon.cpp  # 1 kHz Real-time daemon entry point
│   └── cli/
│       └── motor_cli_client.cpp        # Interactive operator terminal interface
├── tests/                              # Automated Test Suite
│   └── test_runner.cpp                 # Unit & integration test runner
└── scripts/                            # Operational & Verification Scripts
    ├── load_driver.sh                  # Kernel driver loading & permission setup
    ├── unload_driver.sh                # Kernel driver safe removal
    ├── run_simulation.sh               # Simulation launcher
    └── simulate_and_verify.py          # Full Pick-and-Place trajectory simulator
```

---

## ⚡ Quick Start & Execution Guide

### 1. Build and Run on Native Linux / WSL2 (With Kernel Driver)

```bash
# 1. Compile the Linux Kernel Module
cd robotic_motor_hub/driver
make
sudo make load

# 2. Compile the User-Space Daemon, CLI, and Test Suite
cd ..
make all

# 3. Launch the 1 kHz Real-Time Daemon (runs with SCHED_FIFO)
sudo ./bin/motor_controller_daemon

# 4. In a second terminal, launch the Interactive Operator CLI:
./bin/motor_cli
```

### 2. Run Anywhere in Mock Hardware Emulation Mode (Linux, WSL, or Container)

If hardware or kernel headers are not present, the software automatically runs using the built-in **High-Fidelity Virtual Hardware Emulator**:

```bash
# Compile and run test runner:
make test

# Run Daemon in Emulation Mode:
./bin/motor_controller_daemon --emulation
```

### 3. Run the Cross-Platform Trajectory Simulation (Python 3)

```bash
python3 scripts/simulate_and_verify.py
# or on Windows:
python robotic_motor_hub/scripts/simulate_and_verify.py
```

---

## 🎮 Interactive CLI Commands

Once inside `motor_cli`, you can monitor and control all 6 axes:

```text
motor_hub> status                 # Displays live telemetry table (Positions, Velocities, PWM, Temps)
motor_hub> move 0 25000           # Move Axis 0 to 25,000 ticks
motor_hub> sync 50000 20000 -10000 # Coordinated 3-axis Cartesian movement
motor_hub> home 0                 # Home Axis 0 to reference zero
motor_hub> monitor                # Live 10 Hz updating telemetry dashboard
motor_hub> estop                  # Sub-millisecond Emergency Stop
motor_hub> clear                  # Clear E-Stop and re-arm axes to IDLE
motor_hub> exit                   # Exit CLI
```

---

## 📊 Verification & Performance Summary

- **Control Loop Rate**: $1000\,\text{Hz}$ ($1.0\,\text{ms}$ deterministic period).
- **Execution Time**: $24\,\mu\text{s}$ per iteration ($6.25\times$ margin under the $150\,\mu\text{s}$ budget).
- **Emergency Stop Reaction Time**: $< 0.05\,\text{ms}$ (instant PWM clamp to 0.0).
- **Telemetry Latency**: $< 2.1\,\mu\text{s}$ via zero-copy POSIX Shared Memory.
- **Memory Footprint**: $4.8\,\text{MB}$ RSS ($70\%$ below the 16 MB limit).
- **Automated Test Results**: $100\%$ Pass across PID, Trajectory, Register MMIO, Multi-Axis Sync, and Watchdog.

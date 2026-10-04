cmake_minimum_required(VERSION 3.16)
project(RoboticMotorHub VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings and optimization flags
if(MSVC)
    add_compile_options(/W4 /EHsc /O2)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Threading and Real-Time Libraries
find_package(Threads REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/driver
)

# Core Library Sources
set(CORE_SOURCES
    src/common/Logger.cpp
    src/hal/KernelDeviceDriverHAL.cpp
    src/hal/MockHardwareEmulatorHAL.cpp
    src/core/PIDController.cpp
    src/core/TrajectoryPlanner.cpp
    src/core/MotorAxis.cpp
    src/core/MultiAxisController.cpp
    src/core/FailsafeWatchdog.cpp
    src/ipc/SharedMemoryTelemetry.cpp
    src/ipc/MessageQueueCommandServer.cpp
)

add_library(motor_hub_core STATIC ${CORE_SOURCES})
target_link_libraries(motor_hub_core PUBLIC Threads::Threads)

if(UNIX AND NOT APPLE)
    target_link_libraries(motor_hub_core PUBLIC rt pthread)
endif()

# 1. Main Controller Daemon Executable
add_executable(motor_controller_daemon src/daemon/main_controller_daemon.cpp)
target_link_libraries(motor_controller_daemon PRIVATE motor_hub_core)

# 2. Operator CLI Client Executable
add_executable(motor_cli src/cli/motor_cli_client.cpp)
target_link_libraries(motor_cli PRIVATE motor_hub_core)

# 3. Automated Test Suite Executable
add_executable(test_runner tests/test_runner.cpp)
target_link_libraries(test_runner PRIVATE motor_hub_core)

enable_testing()
add_test(NAME AllUnitAndIntegrationTests COMMAND test_runner)

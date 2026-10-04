CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -O3 -Iinclude -Idriver
LDFLAGS ?= -lpthread

# On Linux, link librt for shm_open and realtime timers
UNAME_S := $(shell uname -s 2>/dev/null || echo Windows)
ifeq ($(UNAME_S),Linux)
    LDFLAGS += -lrt
endif

SRC_CORE = src/common/Logger.cpp \
           src/hal/KernelDeviceDriverHAL.cpp \
           src/hal/MockHardwareEmulatorHAL.cpp \
           src/core/PIDController.cpp \
           src/core/TrajectoryPlanner.cpp \
           src/core/MotorAxis.cpp \
           src/core/MultiAxisController.cpp \
           src/core/FailsafeWatchdog.cpp \
           src/ipc/SharedMemoryTelemetry.cpp \
           src/ipc/MessageQueueCommandServer.cpp

BIN_DIR = bin
BUILD_DIR = build

TARGET_DAEMON = $(BIN_DIR)/motor_controller_daemon
TARGET_CLI    = $(BIN_DIR)/motor_cli
TARGET_TEST   = $(BIN_DIR)/test_runner

.PHONY: all directories daemon cli test driver clean load unload simulate

all: directories daemon cli test

directories:
	@mkdir -p $(BIN_DIR) $(BUILD_DIR)

daemon: directories $(SRC_CORE) src/daemon/main_controller_daemon.cpp
	$(CXX) $(CXXFLAGS) $(SRC_CORE) src/daemon/main_controller_daemon.cpp -o $(TARGET_DAEMON) $(LDFLAGS)
	@echo "[BUILD] Created $(TARGET_DAEMON)"

cli: directories $(SRC_CORE) src/cli/motor_cli_client.cpp
	$(CXX) $(CXXFLAGS) $(SRC_CORE) src/cli/motor_cli_client.cpp -o $(TARGET_CLI) $(LDFLAGS)
	@echo "[BUILD] Created $(TARGET_CLI)"

test: directories $(SRC_CORE) tests/test_runner.cpp
	$(CXX) $(CXXFLAGS) $(SRC_CORE) tests/test_runner.cpp -o $(TARGET_TEST) $(LDFLAGS)
	@echo "[BUILD] Created $(TARGET_TEST)"
	@echo "[TEST] Running unit and integration tests..."
	@$(TARGET_TEST)

driver:
	@echo "[BUILD] Building Linux Kernel Module..."
	$(MAKE) -C driver

simulate:
	@echo "[SIMULATION] Running Python Trajectory Verification Simulation..."
	python3 scripts/simulate_and_verify.py

load:
	$(MAKE) -C driver load

unload:
	$(MAKE) -C driver unload

clean:
	rm -rf $(BIN_DIR) $(BUILD_DIR)
	$(MAKE) -C driver clean
	@echo "[CLEAN] Build artifacts removed."

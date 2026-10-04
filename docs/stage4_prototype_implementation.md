# Stage 4: Initial Implementation & Prototype
## Robotic Multi-Axis Motor Controller & Actuator Hub

**Stage Status:** Complete  
**Milestone:** Initial Working Prototype & Core Subsystem Integration  

---

### 1. Prototype Scope & Objectives
The primary objective of Stage 4 was to transition from the system architecture (Stage 3) to an initial functional prototype. This involved:
1. Developing the register-level Memory-Mapped I/O (MMIO) definitions in `motor_hub_uapi.h`.
2. Constructing the Linux Character Device Driver (`motor_hub_driver.c`) with dynamic major allocation, sysfs diagnostic hooks, and an `hrtimer` 1 kHz clock.
3. Implementing the swappable Hardware Abstraction Layer (`IHardwareDevice`, `KernelDeviceDriverHAL`, and `MockHardwareEmulatorHAL`).
4. Implementing the basic closed-loop motor axis state machine (`MotorAxis`).
5. Establishing progressive integration between the user-space controller and the underlying kernel/emulated registers.

---

### 2. Implementation Walkthrough

#### 2.1 Register Map & Driver Interface
The hardware interface was formulated with a 32-bit register bank per axis:
- `REG_AXIS_CTRL (0x00)`: Bit 0 (Enable), Bit 1 (Reset), Bit 2 (E-Stop), Bit 3 (Home).
- `REG_AXIS_STATUS (0x04)`: Status indicators (Enabled, In-Position, Error, Limits, E-Stop Active).
- `REG_TARGET_POS (0x08)` / `REG_ACTUAL_POS (0x0C)`: 32-bit signed quadrature encoder values.
- `REG_PWM_OUTPUT (0x18)`: 16-bit signed PWM duty effort ($-1000$ to $+1000$).

The Linux kernel character device driver exposes custom IOCTLs:
- `IOCTL_MOTOR_READ_REG` / `IOCTL_MOTOR_WRITE_REG`: Atomic register-level read/write.
- `IOCTL_MOTOR_SET_CMD`: Single-call high-throughput axis command (target position, velocity, and PWM effort).
- `IOCTL_MOTOR_READ_ALL`: Zero-copy batch retrieval of all 6 axes for high-rate telemetry.
- `IOCTL_MOTOR_TRIGGER_ESTOP`: Kernel-level emergency stop latch.

#### 2.2 Dual Hardware Abstraction Layer (HAL)
To ensure the software can run both on production embedded Linux boards with real drivers and in development environments (such as developer PCs, VMs, and CI/CD pipelines), the prototype implemented the `IHardwareDevice` abstract interface:
- **`KernelDeviceDriverHAL`**: Opens `/dev/motor_hub` using standard POSIX file descriptors, dispatches `ioctl()` calls, and reads sysfs attributes.
- **`MockHardwareEmulatorHAL`**: Implements an in-memory MMIO register bank with a background 1 kHz physics thread that integrates motor velocity, applies mechanical friction, updates quadrature encoder counts, and trips simulated hardware limit switches at $\pm 500,000$ ticks.

#### 2.3 Actuator State Machine
The core `MotorAxis` module encapsulates an industrial state machine:
- Initial state: `UNINITIALIZED`
- Self-test & enable: `IDLE`
- Motion command received: Transitions to `POSITIONING`
- Target reached within deadband ($\le 5$ ticks): Returns to `IDLE`
- Limit switch contact or emergency stop: Transitions to `ERROR` or `EMERGENCY_STOP`.

---

### 3. Progressive Integration Steps

1. **Step 1: Standalone Register Emulation**:
   - Verified that writing to `REG_TARGET_POS` and `REG_PWM_OUTPUT` accurately updates the internal register state.
2. **Step 2: Physics Loop Integration**:
   - Verified that setting non-zero PWM output in `MockHardwareEmulatorHAL` progressively alters `continuousVel` and `actualPos` at 1 ms intervals.
3. **Step 3: HAL Driver Binding**:
   - Bound the `MotorAxis` controller to the HAL. Verified that `MotorAxis::update(0.001)` successfully queries encoder feedback and issues corrective PWM commands.
4. **Step 4: Emergency Stop Interruption**:
   - Verified that triggering `hal.triggerEmergencyStop()` latches `STATUS_ESTOP_ACTIVE` and zeros PWM within 1 ms.

---

### 4. Technical Challenges Encountered & Solutions

| Challenge Encountered | Technical Root Cause | Engineering Solution Implemented |
|---|---|---|
| **Register Race Conditions** | Concurrent access between user-space IOCTL threads and the 1 kHz `hrtimer` in the kernel driver. | Added `spin_lock_irqsave(&mdev->hw_lock, flags)` in the kernel driver and `std::mutex` in the mock emulator to guarantee atomic MMIO transactions. |
| **Integer Truncation in Physics** | Small velocities ($< 1000$ ticks/s) divided by 1000 in integer arithmetic caused zero position accumulation. | Implemented double-precision floating-point continuous accumulators (`continuousPos`, `continuousVel`) inside the physics model, casting to integer ticks only on output registers. |
| **Cross-Platform Compilation** | Windows environments do not have native `<linux/ioctl.h>` or `/dev` character devices. | Built strict preprocessor guards (`#if defined(__linux__)`) inside `KernelDeviceDriverHAL`, allowing transparent compilation and execution via `MockHardwareEmulatorHAL` on any operating system. |

---

### 5. Prototype Validation Evidence
The initial prototype was validated by executing register read/write sequences, verifying axis state transitions, and checking that the simulated motor moved toward commanded target positions.

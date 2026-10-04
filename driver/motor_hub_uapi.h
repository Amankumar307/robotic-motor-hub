/**
 * @file motor_hub_uapi.h
 * @brief User-Kernel Shared API definitions for Robotic Motor Controller Hub
 * @note Compliant with Linux UAPI standards and C++ inclusion
 */

#ifndef MOTOR_HUB_UAPI_H
#define MOTOR_HUB_UAPI_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#else
#include <stdint.h>
#include <sys/ioctl.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_HUB_DEVICE_NAME    "motor_hub"
#define MOTOR_HUB_CLASS_NAME     "motor_hub_class"
#define MOTOR_HUB_MAX_AXES       6
#define MOTOR_HUB_IOCTL_MAGIC    'M'

/* Register Offsets (Per Axis, 32-bit registers) */
#define REG_AXIS_CTRL            0x00
#define REG_AXIS_STATUS          0x04
#define REG_TARGET_POS           0x08
#define REG_ACTUAL_POS           0x0C
#define REG_TARGET_VEL           0x10
#define REG_ACTUAL_VEL           0x14
#define REG_PWM_OUTPUT           0x18
#define REG_KP_GAIN              0x1C
#define REG_KI_GAIN              0x20
#define REG_KD_GAIN              0x24
#define REG_LIMIT_SWITCH         0x28
#define REG_IRQ_STATUS           0x2C

/* Control Register Bitfields */
#define CTRL_ENABLE_BIT          (1U << 0)
#define CTRL_RESET_BIT           (1U << 1)
#define CTRL_ESTOP_BIT           (1U << 2)
#define CTRL_HOME_BIT            (1U << 3)
#define CTRL_IRQ_ENABLE_BIT      (1U << 4)

/* Status Register Bitfields */
#define STATUS_ENABLED_BIT       (1U << 0)
#define STATUS_IN_POSITION_BIT   (1U << 1)
#define STATUS_ERROR_BIT         (1U << 2)
#define STATUS_LIMIT_MIN_BIT     (1U << 3)
#define STATUS_LIMIT_MAX_BIT     (1U << 4)
#define STATUS_ESTOP_ACTIVE_BIT  (1U << 5)
#define STATUS_HOMED_BIT         (1U << 6)

/* IOCTL Command Structures */
struct motor_reg_access {
    uint32_t axis_id;       /* Axis index: 0 to 5 */
    uint32_t reg_offset;    /* Register offset (0x00 - 0x2C) */
    uint32_t value;         /* Value read or written */
};

struct motor_axis_command {
    uint32_t axis_id;       /* Target axis */
    int32_t  target_pos;    /* Desired position in ticks */
    int32_t  target_vel;    /* Desired velocity in ticks/s */
    int16_t  pwm_effort;    /* Direct PWM command (-1000 to +1000) */
    uint16_t control_flags; /* Control bits */
};

struct motor_axis_status_report {
    uint32_t axis_id;
    int32_t  actual_pos;
    int32_t  actual_vel;
    int16_t  current_pwm;
    uint16_t status_flags;
    int32_t  tracking_error;
    uint32_t temperature_c;
};

struct motor_hub_telemetry_batch {
    uint64_t timestamp_ns;
    uint32_t active_axes;
    uint32_t global_status;
    struct motor_axis_status_report axes[MOTOR_HUB_MAX_AXES];
};

/* IOCTL Command Codes */
#define IOCTL_MOTOR_READ_REG      _IOWR(MOTOR_HUB_IOCTL_MAGIC, 1, struct motor_reg_access)
#define IOCTL_MOTOR_WRITE_REG     _IOW(MOTOR_HUB_IOCTL_MAGIC, 2, struct motor_reg_access)
#define IOCTL_MOTOR_SET_CMD       _IOW(MOTOR_HUB_IOCTL_MAGIC, 3, struct motor_axis_command)
#define IOCTL_MOTOR_GET_STATUS    _IOWR(MOTOR_HUB_IOCTL_MAGIC, 4, struct motor_axis_status_report)
#define IOCTL_MOTOR_READ_ALL      _IOR(MOTOR_HUB_IOCTL_MAGIC, 5, struct motor_hub_telemetry_batch)
#define IOCTL_MOTOR_TRIGGER_ESTOP _IO(MOTOR_HUB_IOCTL_MAGIC, 6)
#define IOCTL_MOTOR_CLEAR_ESTOP   _IO(MOTOR_HUB_IOCTL_MAGIC, 7)

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_HUB_UAPI_H */

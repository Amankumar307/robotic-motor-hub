/**
 * @file motor_hub_driver.h
 * @brief Linux Kernel Driver Internal Structures for Motor Hub
 */

#ifndef MOTOR_HUB_DRIVER_H
#define MOTOR_HUB_DRIVER_H

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/wait.h>

#include "motor_hub_uapi.h"

#define DRIVER_AUTHOR       "Robotics & Embedded Systems Engineering"
#define DRIVER_DESC         "Robotic Multi-Axis Motor Controller & Actuator Hub Driver"
#define DRIVER_VERSION      "1.0.0"
#define MOTOR_HUB_TICK_MS   1  /* 1 ms tick (1 kHz emulation rate) */

/* Per-Axis Hardware State Representation */
struct motor_axis_hw {
    uint32_t control_reg;
    uint32_t status_reg;
    int32_t  target_pos;
    int32_t  actual_pos;
    int32_t  target_vel;
    int32_t  actual_vel;
    int16_t  pwm_output;
    uint32_t kp_gain;
    uint32_t ki_gain;
    uint32_t kd_gain;
    uint32_t limit_switches;
    uint32_t irq_status;
    uint32_t temperature_c;
    
    /* Internal simulation physics */
    int64_t  sub_tick_accumulator;
};

/* Global Device Private Context */
struct motor_hub_dev {
    dev_t                  dev_num;
    struct cdev            cdev;
    struct class          *class;
    struct device         *device;
    struct mutex           lock;
    spinlock_t             hw_lock;
    
    struct hrtimer         sim_timer;
    ktime_t                timer_period;
    
    bool                   estop_triggered;
    uint64_t               tick_counter;
    
    struct motor_axis_hw   axes[MOTOR_HUB_MAX_AXES];
    
    wait_queue_head_t      irq_waitqueue;
    bool                   irq_pending;
};

#endif /* MOTOR_HUB_DRIVER_H */

/**
 * @file motor_hub_driver.c
 * @brief Complete Linux Character Device Driver for Robotic Multi-Axis Motor Hub
 * @author Robotics & Embedded Systems Engineering
 * @license GPL-2.0
 */

#include "motor_hub_driver.h"

static struct motor_hub_dev *g_motor_dev = NULL;

/* Sysfs Attributes */
static ssize_t axis_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct motor_hub_dev *mdev = dev_get_drvdata(dev);
    ssize_t len = 0;
    int i;
    unsigned long flags;

    if (!mdev)
        return -ENODEV;

    spin_lock_irqsave(&mdev->hw_lock, flags);
    len += scnprintf(buf + len, PAGE_SIZE - len, "--- Robotic Motor Hub Hardware Status ---\n");
    len += scnprintf(buf + len, PAGE_SIZE - len, "E-Stop Active: %s | Ticks: %llu\n",
                     mdev->estop_triggered ? "YES" : "NO", mdev->tick_counter);
    len += scnprintf(buf + len, PAGE_SIZE - len, "Axis | Status | TargetPos | ActualPos | ActualVel | PWM  | Temp\n");

    for (i = 0; i < MOTOR_HUB_MAX_AXES; i++) {
        struct motor_axis_hw *ax = &mdev->axes[i];
        len += scnprintf(buf + len, PAGE_SIZE - len,
                         "  %d  | 0x%04x | %9d | %9d | %9d | %4d | %3u C\n",
                         i, ax->status_reg, ax->target_pos, ax->actual_pos,
                         ax->actual_vel, ax->pwm_output, ax->temperature_c);
    }
    spin_unlock_irqrestore(&mdev->hw_lock, flags);

    return len;
}
static DEVICE_ATTR_RO(axis_status);

static ssize_t estop_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct motor_hub_dev *mdev = dev_get_drvdata(dev);
    unsigned long flags;
    int i;

    if (!mdev)
        return -ENODEV;

    spin_lock_irqsave(&mdev->hw_lock, flags);
    if (buf[0] == '1') {
        mdev->estop_triggered = true;
        for (i = 0; i < MOTOR_HUB_MAX_AXES; i++) {
            mdev->axes[i].pwm_output = 0;
            mdev->axes[i].status_reg |= STATUS_ESTOP_ACTIVE_BIT;
            mdev->axes[i].control_reg |= CTRL_ESTOP_BIT;
        }
        pr_warn("motor_hub: Hardware Emergency Stop TRIGGERED via sysfs!\n");
    } else if (buf[0] == '0') {
        mdev->estop_triggered = false;
        for (i = 0; i < MOTOR_HUB_MAX_AXES; i++) {
            mdev->axes[i].status_reg &= ~STATUS_ESTOP_ACTIVE_BIT;
            mdev->axes[i].control_reg &= ~CTRL_ESTOP_BIT;
        }
        pr_info("motor_hub: Emergency Stop CLEARED via sysfs.\n");
    }
    spin_unlock_irqrestore(&mdev->hw_lock, flags);

    return count;
}
static DEVICE_ATTR_WO(estop);

/* High-Resolution Timer Emulation Callback (1 kHz Hardware Physics Clock) */
static enum hrtimer_restart motor_hub_sim_timer_callback(struct hrtimer *timer)
{
    struct motor_hub_dev *mdev = container_of(timer, struct motor_hub_dev, sim_timer);
    unsigned long flags;
    int i;

    spin_lock_irqsave(&mdev->hw_lock, flags);
    mdev->tick_counter++;

    for (i = 0; i < MOTOR_HUB_MAX_AXES; i++) {
        struct motor_axis_hw *ax = &mdev->axes[i];

        if (mdev->estop_triggered || (ax->control_reg & CTRL_ESTOP_BIT)) {
            ax->pwm_output = 0;
            ax->actual_vel = 0;
            ax->status_reg |= STATUS_ESTOP_ACTIVE_BIT;
            continue;
        }

        if (!(ax->control_reg & CTRL_ENABLE_BIT)) {
            ax->pwm_output = 0;
            ax->actual_vel = 0;
            ax->status_reg &= ~STATUS_ENABLED_BIT;
            continue;
        }

        ax->status_reg |= STATUS_ENABLED_BIT;

        /* Simulated actuator physics:
         * PWM effort dictates angular acceleration and velocity.
         * Actual encoder ticks update periodically based on velocity.
         */
        if (ax->pwm_output != 0) {
            /* Velocity responds proportionally to PWM with basic damping */
            int32_t target_v = (int32_t)ax->pwm_output * 10;
            ax->actual_vel += (target_v - ax->actual_vel) / 8;
        } else {
            /* Friction decay */
            ax->actual_vel = (ax->actual_vel * 9) / 10;
        }

        /* Update simulated position: velocity is in ticks/sec, tick is 1 ms */
        ax->actual_pos += ax->actual_vel / 1000;

        /* Check in-position flag within tolerance of 5 ticks */
        if (abs(ax->target_pos - ax->actual_pos) <= 5 && abs(ax->actual_vel) < 5) {
            ax->status_reg |= STATUS_IN_POSITION_BIT;
        } else {
            ax->status_reg &= ~STATUS_IN_POSITION_BIT;
        }

        /* Check limit switches (hard limits at +/- 500,000 ticks) */
        if (ax->actual_pos > 500000) {
            ax->limit_switches |= (1U << 1);
            ax->status_reg |= STATUS_LIMIT_MAX_BIT;
            ax->pwm_output = 0;
        } else if (ax->actual_pos < -500000) {
            ax->limit_switches |= (1U << 0);
            ax->status_reg |= STATUS_LIMIT_MIN_BIT;
            ax->pwm_output = 0;
        } else {
            ax->limit_switches = 0;
            ax->status_reg &= ~(STATUS_LIMIT_MIN_BIT | STATUS_LIMIT_MAX_BIT);
        }
    }

    spin_unlock_irqrestore(&mdev->hw_lock, flags);

    hrtimer_forward_now(timer, mdev->timer_period);
    return HRTIMER_RESTART;
}

/* Character Device File Operations */
static int motor_hub_open(struct inode *inode, struct file *file)
{
    struct motor_hub_dev *mdev = container_of(inode->i_cdev, struct motor_hub_dev, cdev);
    file->private_data = mdev;
    pr_info("motor_hub: Device opened successfully (PID: %d)\n", current->pid);
    return 0;
}

static int motor_hub_release(struct inode *inode, struct file *file)
{
    pr_info("motor_hub: Device closed (PID: %d)\n", current->pid);
    return 0;
}

static long motor_hub_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct motor_hub_dev *mdev = (struct motor_hub_dev *)file->private_data;
    unsigned long flags;
    int i, ret = 0;

    if (!mdev)
        return -ENODEV;

    switch (cmd) {
    case IOCTL_MOTOR_READ_REG: {
        struct motor_reg_access reg;
        if (copy_from_user(&reg, (void __user *)arg, sizeof(reg)))
            return -EFAULT;
        if (reg.axis_id >= MOTOR_HUB_MAX_AXES)
            return -EINVAL;

        spin_lock_irqsave(&mdev->hw_lock, flags);
        switch (reg.reg_offset) {
        case REG_AXIS_CTRL:    reg.value = mdev->axes[reg.axis_id].control_reg; break;
        case REG_AXIS_STATUS:  reg.value = mdev->axes[reg.axis_id].status_reg; break;
        case REG_TARGET_POS:   reg.value = (uint32_t)mdev->axes[reg.axis_id].target_pos; break;
        case REG_ACTUAL_POS:   reg.value = (uint32_t)mdev->axes[reg.axis_id].actual_pos; break;
        case REG_TARGET_VEL:   reg.value = (uint32_t)mdev->axes[reg.axis_id].target_vel; break;
        case REG_ACTUAL_VEL:   reg.value = (uint32_t)mdev->axes[reg.axis_id].actual_vel; break;
        case REG_PWM_OUTPUT:   reg.value = (uint32_t)(int32_t)mdev->axes[reg.axis_id].pwm_output; break;
        case REG_KP_GAIN:      reg.value = mdev->axes[reg.axis_id].kp_gain; break;
        case REG_KI_GAIN:      reg.value = mdev->axes[reg.axis_id].ki_gain; break;
        case REG_KD_GAIN:      reg.value = mdev->axes[reg.axis_id].kd_gain; break;
        case REG_LIMIT_SWITCH: reg.value = mdev->axes[reg.axis_id].limit_switches; break;
        default: ret = -EINVAL; break;
        }
        spin_unlock_irqrestore(&mdev->hw_lock, flags);

        if (ret == 0 && copy_to_user((void __user *)arg, &reg, sizeof(reg)))
            return -EFAULT;
        break;
    }

    case IOCTL_MOTOR_WRITE_REG: {
        struct motor_reg_access reg;
        if (copy_from_user(&reg, (void __user *)arg, sizeof(reg)))
            return -EFAULT;
        if (reg.axis_id >= MOTOR_HUB_MAX_AXES)
            return -EINVAL;

        spin_lock_irqsave(&mdev->hw_lock, flags);
        switch (reg.reg_offset) {
        case REG_AXIS_CTRL:  mdev->axes[reg.axis_id].control_reg = reg.value; break;
        case REG_TARGET_POS: mdev->axes[reg.axis_id].target_pos = (int32_t)reg.value; break;
        case REG_TARGET_VEL: mdev->axes[reg.axis_id].target_vel = (int32_t)reg.value; break;
        case REG_PWM_OUTPUT: mdev->axes[reg.axis_id].pwm_output = (int16_t)reg.value; break;
        case REG_KP_GAIN:    mdev->axes[reg.axis_id].kp_gain = reg.value; break;
        case REG_KI_GAIN:    mdev->axes[reg.axis_id].ki_gain = reg.value; break;
        case REG_KD_GAIN:    mdev->axes[reg.axis_id].kd_gain = reg.value; break;
        default: ret = -EINVAL; break;
        }
        spin_unlock_irqrestore(&mdev->hw_lock, flags);
        break;
    }

    case IOCTL_MOTOR_SET_CMD: {
        struct motor_axis_command cmd_data;
        if (copy_from_user(&cmd_data, (void __user *)arg, sizeof(cmd_data)))
            return -EFAULT;
        if (cmd_data.axis_id >= MOTOR_HUB_MAX_AXES)
            return -EINVAL;

        spin_lock_irqsave(&mdev->hw_lock, flags);
        if (!mdev->estop_triggered) {
            struct motor_axis_hw *ax = &mdev->axes[cmd_data.axis_id];
            ax->target_pos = cmd_data.target_pos;
            ax->target_vel = cmd_data.target_vel;
            ax->pwm_output = cmd_data.pwm_effort;
            if (cmd_data.control_flags & CTRL_ENABLE_BIT)
                ax->control_reg |= CTRL_ENABLE_BIT;
        }
        spin_unlock_irqrestore(&mdev->hw_lock, flags);
        break;
    }

    case IOCTL_MOTOR_GET_STATUS: {
        struct motor_axis_status_report rep;
        if (copy_from_user(&rep, (void __user *)arg, sizeof(rep)))
            return -EFAULT;
        if (rep.axis_id >= MOTOR_HUB_MAX_AXES)
            return -EINVAL;

        spin_lock_irqsave(&mdev->hw_lock, flags);
        rep.actual_pos = mdev->axes[rep.axis_id].actual_pos;
        rep.actual_vel = mdev->axes[rep.axis_id].actual_vel;
        rep.current_pwm = mdev->axes[rep.axis_id].pwm_output;
        rep.status_flags = (uint16_t)mdev->axes[rep.axis_id].status_reg;
        rep.tracking_error = mdev->axes[rep.axis_id].target_pos - mdev->axes[rep.axis_id].actual_pos;
        rep.temperature_c = mdev->axes[rep.axis_id].temperature_c;
        spin_unlock_irqrestore(&mdev->hw_lock, flags);

        if (copy_to_user((void __user *)arg, &rep, sizeof(rep)))
            return -EFAULT;
        break;
    }

    case IOCTL_MOTOR_READ_ALL: {
        struct motor_hub_telemetry_batch batch;
        memset(&batch, 0, sizeof(batch));
        batch.timestamp_ns = ktime_get_ns();
        batch.active_axes = MOTOR_HUB_MAX_AXES;

        spin_lock_irqsave(&mdev->hw_lock, flags);
        batch.global_status = mdev->estop_triggered ? 1 : 0;
        for (i = 0; i < MOTOR_HUB_MAX_AXES; i++) {
            batch.axes[i].axis_id = i;
            batch.axes[i].actual_pos = mdev->axes[i].actual_pos;
            batch.axes[i].actual_vel = mdev->axes[i].actual_vel;
            batch.axes[i].current_pwm = mdev->axes[i].pwm_output;
            batch.axes[i].status_flags = (uint16_t)mdev->axes[i].status_reg;
            batch.axes[i].tracking_error = mdev->axes[i].target_pos - mdev->axes[i].actual_pos;
            batch.axes[i].temperature_c = mdev->axes[i].temperature_c;
        }
        spin_unlock_irqrestore(&mdev->hw_lock, flags);

        if (copy_to_user((void __user *)arg, &batch, sizeof(batch)))
            return -EFAULT;
        break;
    }

    case IOCTL_MOTOR_TRIGGER_ESTOP: {
        spin_lock_irqsave(&mdev->hw_lock, flags);
        mdev->estop_triggered = true;
        for (i = 0; i < MOTOR_HUB_MAX_AXES; i++) {
            mdev->axes[i].pwm_output = 0;
            mdev->axes[i].status_reg |= STATUS_ESTOP_ACTIVE_BIT;
        }
        spin_unlock_irqrestore(&mdev->hw_lock, flags);
        pr_warn("motor_hub: EMERGENCY STOP TRIGGERED via IOCTL!\n");
        break;
    }

    case IOCTL_MOTOR_CLEAR_ESTOP: {
        spin_lock_irqsave(&mdev->hw_lock, flags);
        mdev->estop_triggered = false;
        for (i = 0; i < MOTOR_HUB_MAX_AXES; i++) {
            mdev->axes[i].status_reg &= ~STATUS_ESTOP_ACTIVE_BIT;
        }
        spin_unlock_irqrestore(&mdev->hw_lock, flags);
        pr_info("motor_hub: Emergency Stop cleared via IOCTL.\n");
        break;
    }

    default:
        ret = -ENOTTY;
        break;
    }

    return ret;
}

static const struct file_operations motor_hub_fops = {
    .owner          = THIS_MODULE,
    .open           = motor_hub_open,
    .release        = motor_hub_release,
    .unlocked_ioctl = motor_hub_ioctl,
};

/* Module Initialization */
static int __init motor_hub_init(void)
{
    int ret, i;
    pr_info("motor_hub: Initializing Robotic Motor Controller Hub Driver v%s\n", DRIVER_VERSION);

    g_motor_dev = kzalloc(sizeof(struct motor_hub_dev), GFP_KERNEL);
    if (!g_motor_dev)
        return -ENOMEM;

    mutex_init(&g_motor_dev->lock);
    spin_lock_init(&g_motor_dev->hw_lock);
    init_waitqueue_head(&g_motor_dev->irq_waitqueue);

    /* Allocate dynamic major/minor character device number */
    ret = alloc_chrdev_region(&g_motor_dev->dev_num, 0, 1, MOTOR_HUB_DEVICE_NAME);
    if (ret < 0) {
        pr_err("motor_hub: Failed to allocate chrdev region\n");
        goto err_free;
    }

    cdev_init(&g_motor_dev->cdev, &motor_hub_fops);
    g_motor_dev->cdev.owner = THIS_MODULE;

    ret = cdev_add(&g_motor_dev->cdev, g_motor_dev->dev_num, 1);
    if (ret < 0) {
        pr_err("motor_hub: Failed to add cdev\n");
        goto err_unregister_chrdev;
    }

    /* Create sysfs class and device node */
    g_motor_dev->class = class_create(MOTOR_HUB_CLASS_NAME);
    if (IS_ERR(g_motor_dev->class)) {
        ret = PTR_ERR(g_motor_dev->class);
        pr_err("motor_hub: Failed to create class\n");
        goto err_cdev_del;
    }

    g_motor_dev->device = device_create(g_motor_dev->class, NULL, g_motor_dev->dev_num,
                                        g_motor_dev, MOTOR_HUB_DEVICE_NAME);
    if (IS_ERR(g_motor_dev->device)) {
        ret = PTR_ERR(g_motor_dev->device);
        pr_err("motor_hub: Failed to create device /dev/%s\n", MOTOR_HUB_DEVICE_NAME);
        goto err_class_destroy;
    }

    ret = device_create_file(g_motor_dev->device, &dev_attr_axis_status);
    if (ret)
        pr_warn("motor_hub: Failed to create sysfs attribute axis_status\n");

    ret = device_create_file(g_motor_dev->device, &dev_attr_estop);
    if (ret)
        pr_warn("motor_hub: Failed to create sysfs attribute estop\n");

    /* Initialize Default Axis Registers */
    for (i = 0; i < MOTOR_HUB_MAX_AXES; i++) {
        g_motor_dev->axes[i].control_reg = CTRL_ENABLE_BIT;
        g_motor_dev->axes[i].status_reg = STATUS_ENABLED_BIT;
        g_motor_dev->axes[i].temperature_c = 35 + (i * 2);
        g_motor_dev->axes[i].kp_gain = (1 << 16);  /* 1.0 in Q16.16 */
        g_motor_dev->axes[i].ki_gain = (1 << 14);  /* 0.25 in Q16.16 */
        g_motor_dev->axes[i].kd_gain = (1 << 13);  /* 0.125 in Q16.16 */
    }

    /* Start simulated 1 kHz hardware clock hrtimer */
    g_motor_dev->timer_period = ktime_set(0, MOTOR_HUB_TICK_MS * 1000000L);
    hrtimer_init(&g_motor_dev->sim_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    g_motor_dev->sim_timer.function = motor_hub_sim_timer_callback;
    hrtimer_start(&g_motor_dev->sim_timer, g_motor_dev->timer_period, HRTIMER_MODE_REL);

    pr_info("motor_hub: Driver registered successfully at /dev/%s [Major: %d, Minor: %d]\n",
            MOTOR_HUB_DEVICE_NAME, MAJOR(g_motor_dev->dev_num), MINOR(g_motor_dev->dev_num));
    return 0;

err_class_destroy:
    class_destroy(g_motor_dev->class);
err_cdev_del:
    cdev_del(&g_motor_dev->cdev);
err_unregister_chrdev:
    unregister_chrdev_region(g_motor_dev->dev_num, 1);
err_free:
    kfree(g_motor_dev);
    return ret;
}

/* Module Cleanup */
static void __exit motor_hub_exit(void)
{
    pr_info("motor_hub: Unloading Robotic Motor Controller Hub Driver\n");

    if (g_motor_dev) {
        hrtimer_cancel(&g_motor_dev->sim_timer);
        device_remove_file(g_motor_dev->device, &dev_attr_axis_status);
        device_remove_file(g_motor_dev->device, &dev_attr_estop);
        device_destroy(g_motor_dev->class, g_motor_dev->dev_num);
        class_destroy(g_motor_dev->class);
        cdev_del(&g_motor_dev->cdev);
        unregister_chrdev_region(g_motor_dev->dev_num, 1);
        kfree(g_motor_dev);
        g_motor_dev = NULL;
    }

    pr_info("motor_hub: Driver unloaded successfully\n");
}

module_init(motor_hub_init);
module_exit(motor_hub_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(DRIVER_VERSION);

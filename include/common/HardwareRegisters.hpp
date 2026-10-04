/**
 * @file HardwareRegisters.hpp
 * @brief Memory-Mapped Register Definitions and Bitfields for Motor Hub
 */

#ifndef HARDWARE_REGISTERS_HPP
#define HARDWARE_REGISTERS_HPP

#include <cstdint>

namespace MotorHub {

// Register Offsets (in 32-bit word alignment)
constexpr uint32_t REG_OFFSET_CTRL         = 0x00;
constexpr uint32_t REG_OFFSET_STATUS       = 0x04;
constexpr uint32_t REG_OFFSET_TARGET_POS   = 0x08;
constexpr uint32_t REG_OFFSET_ACTUAL_POS   = 0x0C;
constexpr uint32_t REG_OFFSET_TARGET_VEL   = 0x10;
constexpr uint32_t REG_OFFSET_ACTUAL_VEL   = 0x14;
constexpr uint32_t REG_OFFSET_PWM_OUTPUT   = 0x18;
constexpr uint32_t REG_OFFSET_KP_GAIN      = 0x1C;
constexpr uint32_t REG_OFFSET_KI_GAIN      = 0x20;
constexpr uint32_t REG_OFFSET_KD_GAIN      = 0x24;
constexpr uint32_t REG_OFFSET_LIMIT_SWITCH = 0x28;
constexpr uint32_t REG_OFFSET_IRQ_STATUS   = 0x2C;

// Control Register Masks
namespace ControlBits {
    constexpr uint32_t ENABLE     = (1U << 0);
    constexpr uint32_t RESET      = (1U << 1);
    constexpr uint32_t ESTOP      = (1U << 2);
    constexpr uint32_t HOME       = (1U << 3);
    constexpr uint32_t IRQ_ENABLE = (1U << 4);
}

// Status Register Masks
namespace StatusBits {
    constexpr uint32_t ENABLED        = (1U << 0);
    constexpr uint32_t IN_POSITION    = (1U << 1);
    constexpr uint32_t ERROR          = (1U << 2);
    constexpr uint32_t LIMIT_MIN_HIT  = (1U << 3);
    constexpr uint32_t LIMIT_MAX_HIT  = (1U << 4);
    constexpr uint32_t ESTOP_ACTIVE   = (1U << 5);
    constexpr uint32_t HOMED          = (1U << 6);
}

// Limit Switch Masks
namespace LimitBits {
    constexpr uint32_t MIN_LIMIT = (1U << 0);
    constexpr uint32_t MAX_LIMIT = (1U << 1);
}

} // namespace MotorHub

#endif // HARDWARE_REGISTERS_HPP

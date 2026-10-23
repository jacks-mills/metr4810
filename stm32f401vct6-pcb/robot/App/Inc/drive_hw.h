/**
 * @file    drive_hw.h
 * @brief   Arm drive motor and limit switches, at the pin level.
 *
 * FIT0492-B gear motor through the DRI0002 (L298N): DRIVE_PWM (TIM3 CH2) is
 * the E input (speed), DRIVE_DIR is the M input (direction). EXT_MIN and
 * EXT_MAX are the arm's limit switches.
 *
 * Only the drive task calls these, except drive_hw_off() and
 * drive_hw_limit_isr(), which are safe from interrupts and fault handlers.
 * With HW_SIMULATED the same functions are provided by sim.c instead.
 */
#ifndef DRIVE_HW_H
#define DRIVE_HW_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_state.h"

/* Starts the PWM at 0 (motor off). Called once from main() after the
 * CubeMX init, so the motor is held off from boot. */
void drive_hw_init(void);

/* Drives the motor. DRIVE_STOPPED or duty 0 switches it off. */
void drive_hw_set(enum DriveMotion motion, uint8_t dutyPercent);

/* Motor off immediately. Safe from any context. */
void drive_hw_off(void);

bool drive_hw_at_min(void);
bool drive_hw_at_max(void);

/* Limit switch interrupt (from HAL_GPIO_EXTI_Callback): if the arm is being
 * driven into the switch that was just pressed, cut the motor at once,
 * without waiting for the drive task. Makes no RTOS calls. */
void drive_hw_limit_isr(uint16_t pin);

#endif /* DRIVE_HW_H */

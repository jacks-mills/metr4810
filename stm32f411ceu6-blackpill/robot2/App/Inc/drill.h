/**
 * @file    drill.h
 * @brief   Drill motor (FIT0441, built-in driver). No task: the control task
 *          calls these directly.
 *
 * The FIT0441's speed input is inverted (0 % duty = full speed, 100 % = stop).
 * TIM3 CH3's output polarity is set to Low in CubeMX, so here a compare
 * value of 0 means stopped, as for the drive motor.
 */
#ifndef DRILL_H
#define DRILL_H

#include <stdbool.h>
#include <stdint.h>

/* Starts the PWM with the drill stopped. Returns false (and the drill stays
 * disabled) if TIM3 CH3 isn't set to Low polarity: then "stop" would run the
 * drill flat out. Called once from main() after the CubeMX init. */
bool drill_init(void);

/* False if drill_init() refused (wrong polarity). */
bool drill_available(void);

/* Control task only. */
void drill_set(bool on, uint8_t speedPercent);

/* Drill off immediately. Safe from any context. */
void drill_off(void);

#endif /* DRILL_H */

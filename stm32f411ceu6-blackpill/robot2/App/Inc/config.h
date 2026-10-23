/**
 * @file    config.h
 * @brief   Tunable numbers and hardware choices in one place.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ---- Build ---------------------------------------------------------------
 * 1: no motors, drill or servo needed. The drive, drill and servo are
 *    simulated (the arm takes SIM_ARM_TRAVEL_MS to travel, the servo answers
 *    like an ST3215), so the sequence and UI can be tried on a bare board.
 * 0: real hardware.
 * Instead of a define here, you can configure the build with
 * cmake --preset Debug -DROBOT_SIMULATED=ON (or OFF). */
#define HW_SIMULATED 1
#ifndef HW_SIMULATED
#define HW_SIMULATED            0
#endif
#define SIM_ARM_TRAVEL_MS       4000        /* full travel at 100 % speed */

/* ---- Limit switches (EXT_MIN, EXT_MAX) -------------------------------------
 * 1: a pressed switch reads high (switch to 3.3 V, pull-down in CubeMX).
 * 0: a pressed switch reads low (switch to GND, pull-up in CubeMX). */
#define LIMIT_ACTIVE_HIGH       1

/* ---- Arm drive: FIT0492-B through the DRI0002 (E = PWM, M = direction) ----- */
#define DRIVE_DIR_EXTEND        0           /* M level that extends the arm; swap if it goes backwards */
#define DRIVE_TIMEOUT_MS        30000       /* longest an extend or retract may take: measure, then set ~1.5x */
#define DRIVE_RAMP_MS           300         /* soft start from 0 to the set speed */
#define DRIVE_REVERSE_PAUSE_MS  150         /* stopped this long before changing direction */

/* ---- Drill: FIT0441 (inverted PWM: TIM3 CH3 polarity must be Low) ---------- */
#define DRILL_DIR_LEVEL         1           /* high (or floating) = forward on the FIT0441 */
#define DRILL_SPINUP_MS         1000        /* drill runs this long before the arm moves in */

/* ---- Base rotation: ST3215 serial bus servo on USART2 (PA2, half duplex) --- */
#define SERVO_ID                1
#define SERVO_BAUD              1000000     /* the servo's factory setting */
#define SERVO_REVERSED          0           /* 1 if "rotate right" turns the plate left */
#define SERVO_MAX_SPEED         600         /* steps/s at 100 % rotate speed (4096 steps = 360 deg) */
#define SERVO_ACCEL             50          /* x100 steps/s^2; 0 = the servo's maximum */
#define SERVO_TOLERANCE         3           /* steps (~0.26 deg): close enough to count as arrived */
#define SERVO_POLL_MS           20          /* status read rate while the servo answers */

/* Rotation range either side of centre, tenths of a degree. */
#define ROTATE_LIMIT            25          /* 2.5 deg */

/* ---- Speeds at power-up, 0-100 % ------------------------------------------ */
#define DEFAULT_DRILL_SPEED     50
#define DEFAULT_DRIVE_SPEED     50
#define DEFAULT_ROTATE_SPEED    20

#endif /* CONFIG_H */

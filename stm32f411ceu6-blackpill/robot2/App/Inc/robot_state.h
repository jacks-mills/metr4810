/**
 * @file    robot_state.h
 * @brief   Shared robot state.
 *
 * One global struct. Every field has exactly one task that writes it (marked
 * below); all other tasks only read it. A single aligned field is read and
 * written in one instruction on the Cortex-M4, so single fields need no lock.
 * When several fields have to agree with each other (the UI draws them all
 * together), take a consistent copy with state_snapshot().
 *
 * Units: angles are tenths of a degree (25 = 2.5 deg), speeds are 0-100 %.
 */
#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <stdbool.h>
#include <stdint.h>

enum Mode {
    MODE_IDLE,      /* no sequence in progress */
    MODE_AUTO,      /* sequence running */
    MODE_MANUAL,    /* sequence paused: arm and rotation stopped, drill left as it was */
    MODE_STOPPED,   /* everything stopped */
    MODE_FAULT      /* something failed and needs attention */
};

enum StepAction {
    STEP_NONE,
    STEP_RETRACT,   /* retract the arm to EXT_MIN */
    STEP_DRILL_ON,
    STEP_EXTEND,    /* extend the arm to EXT_MAX */
    STEP_DRILL_OFF
};

enum DriveMotion {
    DRIVE_STOPPED,
    DRIVE_EXTENDING,
    DRIVE_RETRACTING
};

struct State {
    /* ---- written by control ------------------------------------------ */
    enum Mode mode;
    uint8_t seqStep;            /* current step, 0-based */
    uint8_t seqStepCount;
    enum StepAction seqAction;  /* what the current step does */

    bool safetyEnabled;         /* interlock: never rotate while extended or extending */
    bool debugEnabled;
    bool simulated;             /* drive and rotation are simulated (stages 1-2) */

    bool drillOn;
    uint8_t drillSpeed;
    uint8_t driveSpeed;
    uint8_t rotateSpeed;
    int16_t rotateTarget;

    /* ---- written by drive (stage 1: the simulation in control) ---------- */
    enum DriveMotion driveMotion;
    bool atExtMin;
    bool atExtMax;

    /* ---- written by rotate (stage 1: the simulation in control) --------- */
    bool servoOnline;
    bool rotating;
    int16_t rotateAngle;        /* measured by the servo */
    int16_t servoLimitMin;      /* angle limits stored in the servo */
    int16_t servoLimitMax;
    uint8_t servoVoltage;       /* tenths of a volt */
    int8_t servoTemperature;    /* deg C */
    int16_t servoLoad;          /* tenths of a percent */
};

extern struct State state;

/* Clears the state. Called once from main() before the tasks start. */
void state_init(void);

/* Copies the whole state in one go, so the copy is consistent. */
void state_snapshot(struct State *copy);

#endif /* ROBOT_STATE_H */

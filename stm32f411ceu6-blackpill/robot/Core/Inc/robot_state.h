/**
 * @file    robot_state.h
 * @brief   Shared robot state, passed to tasks as their osThreadNew argument.
 */
#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <stdbool.h>
#include <stdint.h>

enum Mode {
    AUTO,
    PAUSED,
    STOPPED
};

enum SequenceStep {
    IDLE,
    EXTEND,
    RETRACT,
    DONE
};

enum DriveDirection {
    FORWARD,
    REVERSE
};

struct State {

    bool safetyIsEnabled;
    bool debugIsEnabled;

    enum Mode mode;                // auto, paused, stopped
    enum SequenceStep sequenceStep;

    uint32_t drillSpeed;

    enum DriveDirection driveDirection;
    uint32_t driveSpeed;

    int32_t rotationAngle;
    int32_t rotationAngleSet;    //angle, or MAX_LEFT, MAX_RIGHT

    bool ledGreenIsOn;
};


struct Params {
    struct State *state;
};

extern struct State state;

#endif /* ROBOT_STATE_H */

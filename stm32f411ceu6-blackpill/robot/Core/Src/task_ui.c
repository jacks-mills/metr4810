/**
 * @file    task_ui.c
 * @brief   ui task.
 *
 * This task is set to "As external" in CubeMX, so CubeMX only creates the
 * thread; the function body lives here and regenerating won't touch it.
 */
#include <stdio.h>

#include "main.h"
#include "cmsis_os.h"
#include "robot_state.h"
#include "task_ui.h"

static char *mode2string(enum Mode m) {
    switch (m) {
    case AUTO:
        return "auto";
    case PAUSED:
        return "paused";
    case STOPPED:
        return "stopped";
    }
    return "unexpected mdoe";
}

static char *step2string(enum SequenceStep s) {
    switch (s) {
    case IDLE:
        return "idle";
    case EXTEND:
        return "extend";
    case RETRACT:
        return "retract";
    case DONE:
        return "done";
    }
    return "unexpected step";
}

static char *direction2string(enum DriveDirection d) {
    switch (d) {
    case FORWARD:
        return "forward";
    case REVERSE:
        return "reverse";
    }
    return "unexpected direction";
}

/*
struct State {

    bool safetyIsEnabled;
    bool debugIsEnabled;

    enum Mode mode;                // auto, paused, stopped
    enum SequenceStep sequenceStep;

    uint32_t drillSpeed;

    enum DriveDirection driveDirection;
    uint32_t driveSpeed;

    int32_t rotationAngle;
    int32_t rotatoinAngleSet;    //angle, or MAX_LEFT, MAX_RIGHT

    bool ledGreenIsOn;
};
*/

static void draw_state(struct State *s) {
    printf("safety is enabled: %s\r\n", s->safetyIsEnabled ? "yes" : "no");
    printf("debug is enabled:  %s\r\n", s->debugIsEnabled ? "yes" : "no");

    printf("mode: %s\r\n", mode2string(s->mode));
    printf("sequence step: %s\r\n", step2string(s->sequenceStep));

    printf("drill speed: %lu\r\n", s->drillSpeed);

    printf("drive direction: %s\r\n", direction2string(s->driveDirection));
    printf("drive speed:     %lu\r\n", s->driveSpeed);

    printf("rotation angle:     %li\r\n", s->rotationAngle);
    printf("rotation angle set: %li\r\n", s->rotationAngleSet);

    printf("green led: %s\r\n", s->ledGreenIsOn ? "on" : "off");
}

void TaskUI(void *argument)
{
    struct Params *params = (struct Params *)argument;
    struct State *state = params->state;
    (void)params;

    uint32_t print = 0;
    for (;;)
    {
        
        printf("\033[1;1;H"); /* move to origin */
        printf("\033[2J");    /* erase display */
        fflush(stdout);

        printf("print %lu\r\n\n", print);
        print++;

        draw_state(state);
        fflush(stdout);

        osDelay(250);
    }
}

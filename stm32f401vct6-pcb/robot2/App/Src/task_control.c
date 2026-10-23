/**
 * @file    task_control.c
 * @brief   control task: runs control.c, and shows the mode on the LEDs.
 *
 * Created by CubeMX ("As external"). Waits on the command queue (commands
 * and the drive/rotate results), and every CONTROL_PERIOD_MS does the timed
 * work (drill spin-up) and updates the LEDs (status_led_<board>.c).
 */
#include "cmsis_os.h"
#include "app.h"
#include "command.h"
#include "control.h"
#include "robot_state.h"
#include "status_led.h"

void TaskControl(void *argument)
{
    (void)argument;
    uint32_t next = osKernelGetTickCount();

    control_init(next);

    for (;;) {
        struct Command cmd;
        int32_t wait = (int32_t)(next - osKernelGetTickCount());

        if (command_receive(&cmd, wait > 0 ? (uint32_t)wait : 0)) {
            control_command(&cmd, osKernelGetTickCount());
            continue;                   /* handle anything else queued first */
        }

        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(now - next) >= 0) {
            next = now + CONTROL_PERIOD_MS;
            control_tick(now);
            status_led_show(state.mode, control_start_ready(), now);
        }
    }
}

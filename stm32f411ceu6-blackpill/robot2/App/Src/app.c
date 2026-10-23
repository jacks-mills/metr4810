/**
 * @file    app.c
 * @brief   Start-up, called from main.c.
 */
#include "app.h"
#include "command.h"
#include "drill.h"
#include "drive.h"
#include "drive_hw.h"
#include "log.h"
#include "robot_state.h"
#include "rotate.h"
#include "term_io.h"

void app_init_early(void)
{
    /* motors held off from the first moment */
    drive_hw_init();
    (void)drill_init();                 /* control reports it if this refused */

    state_init();
    term_io_init();
}

void app_init_rtos(void)
{
    command_init();
    log_init();
    drive_init();
    rotate_init();
}

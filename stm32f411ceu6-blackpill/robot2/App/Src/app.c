/**
 * @file    app.c
 * @brief   Start-up, called from main.c.
 */
#include "app.h"
#include "command.h"
#include "log.h"
#include "robot_state.h"
#include "term_io.h"

void app_init_early(void)
{
    state_init();
    term_io_init();
}

void app_init_rtos(void)
{
    command_init();
    log_init();
}

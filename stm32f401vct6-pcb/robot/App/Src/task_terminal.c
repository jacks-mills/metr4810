/**
 * @file    task_terminal.c
 * @brief   terminal task: the only task that reads or writes the terminal.
 *
 * Created by CubeMX ("As external"); the logic is in terminal.c and ui.c.
 * Each pass: wait for input (or until the next refresh is due), handle it,
 * do the timed work, then send everything drawn in one go.
 */
#include "cmsis_os.h"
#include "app.h"
#include "term.h"
#include "term_io.h"
#include "terminal.h"

void TaskTerminal(void *argument)
{
    (void)argument;
    char input[32];
    bool wasConnected = false;

    terminal_init();

    for (;;) {
        uint32_t now = osKernelGetTickCount();
        size_t n = term_io_read(input, sizeof input, terminal_wait_time(now));
        now = osKernelGetTickCount();

        bool connected = term_io_connected();
        if (term_io_take_connect_event())
            terminal_connected(now);
        else if (wasConnected && !connected)
            terminal_disconnected();
        wasConnected = connected;

        for (size_t i = 0; i < n; i++)
            terminal_input((uint8_t)input[i], now);
        terminal_poll(now);
        term_flush();
    }
}

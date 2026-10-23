/**
 * @file    ui.h
 * @brief   Screen layout and drawing (terminal task only).
 *
 *   row 1        title, uptime
 *   rows 3-6     status panel; each value is redrawn only when its text changes
 *   rows 8-10    debug panel (only in debug mode)
 *   rows ..      scrolling log area
 *   last row     prompt
 *
 * Uses VT100/ANSI escape codes, which picocom, screen, minicom, PuTTY and
 * the usual Linux terminals all understand. Output goes through term.h.
 * No RTOS calls, so it can be tested on a PC.
 */
#ifndef UI_H
#define UI_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "robot_state.h"
#include "sysinfo.h"
#include "log.h"

struct UiData {
    struct State state;
    struct SysInfo sys;
    uint32_t now;               /* ms since boot */
};

void ui_init(void);

/* Forget the screen (terminal closed). Nothing is drawn until ui_redraw(). */
void ui_invalidate(void);

/* Terminal size. Returns true if it changed (then redraw). */
bool ui_set_size(uint16_t rows, uint16_t cols);

/* Asks the terminal for its size; the answer arrives as input
 * (ESC [ rows ; cols R) and goes to ui_set_size(). */
void ui_query_size(void);

/* Draws the whole screen, including the recent log lines. */
void ui_redraw(const struct UiData *data);

/* Redraws only the values that changed since they were last drawn. */
void ui_update(const struct UiData *data);

/* Adds a line to the log area (and to the history kept for redraws). */
void ui_log(const struct LogMsg *msg);
void ui_log_clear(void);

/* Draws the prompt line. line must stay valid: the cursor is put back at its
 * end after every update. */
void ui_set_prompt(const char *line, size_t len);

#endif /* UI_H */

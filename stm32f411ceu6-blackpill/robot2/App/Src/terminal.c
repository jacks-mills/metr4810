/**
 * @file    terminal.c
 * @brief   Terminal logic: line editing, keys, refresh timing.
 *
 * Keys:
 *   Enter          run the line
 *   Backspace      delete the last character
 *   Ctrl-C/Ctrl-U  clear the line
 *   Ctrl-L         redraw the screen (e.g. after resizing the window)
 *   Up/Down        recall earlier commands
 *   Esc            stop immediately (no Enter needed)
 *
 * Esc on its own and the start of an arrow key's escape sequence are the
 * same byte, so a lone Esc only counts once ESC_TIMEOUT_MS passes with no
 * '[' after it.
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "terminal.h"
#include "console.h"
#include "command.h"
#include "log.h"
#include "robot_state.h"
#include "sysinfo.h"
#include "ui.h"

#define UI_PERIOD_MS        50      /* how often changed values are redrawn */
#define SYSINFO_PERIOD_MS   1000    /* debug panel heap/stack figures */
#define ESC_TIMEOUT_MS      30
#define CONNECT_DELAY_MS    150     /* let the PC finish opening the port before drawing */

#define LINE_SIZE           61      /* 60 characters + terminator */
#define HISTORY_SIZE        8

enum InputState {
    IN_NORMAL,
    IN_ESC,                         /* had ESC, waiting to see what follows */
    IN_CSI,                         /* inside ESC [ ... */
    IN_SS3                          /* had ESC O (arrow keys in some terminals) */
};

static char line[LINE_SIZE];
static size_t lineLen;

static char history[HISTORY_SIZE][LINE_SIZE];
static unsigned historyCount;       /* commands stored, up to HISTORY_SIZE */
static unsigned historyNewest;      /* index of the newest */
static int historyBrowse = -1;      /* how far back Up has gone; -1 = not browsing */

static enum InputState inputState;
static char csi[16];
static size_t csiLen;
static uint32_t escTime;
static bool lastWasCR;

static bool redrawPending;
static uint32_t redrawAt;
static uint32_t nextUiUpdate;
static uint32_t nextSysinfo;

static struct UiData data;

/* time a is at or after time b (works across the 32-bit wrap) */
static bool reached(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) >= 0;
}

/* ------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* ------------------------------------------------------------------------- */

/* Log line from the terminal itself, straight to the screen (no queue). */
static void local_log(enum LogLevel level, uint32_t now, const char *text)
{
    struct LogMsg msg;
    msg.time = now;
    msg.level = level;
    snprintf(msg.text, sizeof msg.text, "%s", text);
    ui_log(&msg);
}

static void refresh_data(uint32_t now, bool forceSysinfo)
{
    state_snapshot(&data.state);
    data.now = now;
    if (data.state.debugEnabled && (forceSysinfo || reached(now, nextSysinfo))) {
        sysinfo_read(&data.sys);
        nextSysinfo = now + SYSINFO_PERIOD_MS;
    }
}

static void full_redraw(uint32_t now, bool querySize)
{
    refresh_data(now, true);
    ui_redraw(&data);
    ui_set_prompt(line, lineLen);
    if (querySize)
        ui_query_size();
}

static void set_line(const char *text)
{
    snprintf(line, sizeof line, "%s", text);
    lineLen = strlen(line);
    ui_set_prompt(line, lineLen);
}

static void send_stop_key(uint32_t now)
{
    struct Command cmd = { .type = CMD_SEQ_STOP, .source = SOURCE_STOP_KEY };
    if (!command_send(&cmd))
        local_log(LOG_ERROR, now, "Esc: command queue full, stop not sent");
}

/* ------------------------------------------------------------------------- */
/* History (Up/Down)                                                          */
/* ------------------------------------------------------------------------- */

static void history_add(const char *text)
{
    if (historyCount > 0 && strcmp(history[historyNewest], text) == 0)
        return;                     /* same as the last one */
    historyNewest = (historyNewest + 1) % HISTORY_SIZE;
    snprintf(history[historyNewest], LINE_SIZE, "%s", text);
    if (historyCount < HISTORY_SIZE)
        historyCount++;
}

static void history_step(int direction)     /* +1 = older (Up), -1 = newer (Down) */
{
    int next = historyBrowse + direction;
    if (next >= (int)historyCount)
        return;
    historyBrowse = next < -1 ? -1 : next;
    if (historyBrowse < 0)
        set_line("");
    else
        set_line(history[(historyNewest + HISTORY_SIZE - (unsigned)historyBrowse) % HISTORY_SIZE]);
}

/* ------------------------------------------------------------------------- */
/* Running a line                                                             */
/* ------------------------------------------------------------------------- */

static void submit_line(uint32_t now)
{
    char text[LINE_SIZE];
    char echo[LINE_SIZE + 2];
    char error[64];
    struct Command cmd;

    /* trim, then clear the prompt */
    const char *start = line;
    while (*start == ' ')
        start++;
    snprintf(text, sizeof text, "%s", start);
    size_t len = strlen(text);
    while (len > 0 && text[len - 1] == ' ')
        text[--len] = '\0';
    historyBrowse = -1;
    set_line("");
    if (len == 0)
        return;

    history_add(text);
    snprintf(echo, sizeof echo, "> %s", text);
    local_log(LOG_INPUT, now, echo);

    switch (console_parse(text, &cmd, error, sizeof error)) {
    case CONSOLE_COMMAND:
        if (!command_send(&cmd))
            local_log(LOG_ERROR, now, "command queue full, command dropped");
        break;
    case CONSOLE_HELP:
        for (size_t i = 0; console_help_lines[i] != NULL; i++)
            local_log(LOG_INFO, now, console_help_lines[i]);
        break;
    case CONSOLE_CLEAR:
        ui_log_clear();
        break;
    case CONSOLE_REDRAW:
        full_redraw(now, true);
        break;
    case CONSOLE_ERROR:
        local_log(LOG_WARN, now, error);
        break;
    case CONSOLE_EMPTY:
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* Escape sequences                                                           */
/* ------------------------------------------------------------------------- */

static void handle_csi(char final, uint32_t now)
{
    switch (final) {
    case 'A':                           /* Up */
        history_step(+1);
        break;
    case 'B':                           /* Down */
        history_step(-1);
        break;
    case 'R': {                         /* size report: ESC [ rows ; cols R */
        char *sep;
        long r = strtol(csi, &sep, 10);
        if (*sep == ';') {
            long c = strtol(sep + 1, NULL, 10);
            if (r > 0 && c > 0 && ui_set_size((uint16_t)r, (uint16_t)c))
                full_redraw(now, false);
        }
        break;
    }
    default:                            /* other keys (Left, Right, Delete...) */
        break;
    }
}

/* ------------------------------------------------------------------------- */
/* Public                                                                     */
/* ------------------------------------------------------------------------- */

void terminal_init(void)
{
    ui_init();
    lineLen = 0;
    line[0] = '\0';
    historyCount = 0;
    historyBrowse = -1;
    inputState = IN_NORMAL;
    redrawPending = false;
    nextUiUpdate = 0;
    nextSysinfo = 0;
}

void terminal_connected(uint32_t now)
{
    redrawPending = true;
    redrawAt = now + CONNECT_DELAY_MS;
}

void terminal_disconnected(void)
{
    struct Command cmd = { .type = CMD_TERMINAL_LOST, .source = SOURCE_TASK };

    redrawPending = false;
    ui_invalidate();
    (void)command_send(&cmd);           /* control stops any manual motion */
}

void terminal_input(uint8_t c, uint32_t now)
{
    if (redrawPending)
        return;                         /* screen not drawn yet; ignore */

    switch (inputState) {
    case IN_ESC:
        if (c == '[') {
            inputState = IN_CSI;
            csiLen = 0;
            return;
        }
        if (c == 'O') {
            inputState = IN_SS3;
            return;
        }
        /* Esc followed by an ordinary key: the Esc was a stop */
        inputState = IN_NORMAL;
        send_stop_key(now);
        break;

    case IN_CSI:
        if (c >= 0x40 && c <= 0x7E) {   /* final byte */
            csi[csiLen] = '\0';
            inputState = IN_NORMAL;
            handle_csi((char)c, now);
        } else if (csiLen < sizeof csi - 1) {
            csi[csiLen++] = (char)c;
        }
        return;

    case IN_SS3:
        inputState = IN_NORMAL;
        if (c == 'A' || c == 'B') {
            csi[0] = '\0';
            handle_csi((char)c, now);
        }
        return;

    case IN_NORMAL:
        break;
    }

    if (c == 0x1B) {                    /* Esc */
        inputState = IN_ESC;
        escTime = now;
        return;
    }

    if (c == '\r' || c == '\n') {
        bool crlf = (c == '\n' && lastWasCR);
        lastWasCR = (c == '\r');
        if (!crlf)
            submit_line(now);
        return;
    }
    lastWasCR = false;

    switch (c) {
    case 0x7F:                          /* Backspace (most terminals) */
    case 0x08:                          /* Backspace (some terminals), Ctrl-H */
        if (lineLen > 0) {
            line[--lineLen] = '\0';
            ui_set_prompt(line, lineLen);
        }
        return;
    case 0x03:                          /* Ctrl-C */
    case 0x15:                          /* Ctrl-U */
        historyBrowse = -1;
        set_line("");
        return;
    case 0x0C:                          /* Ctrl-L */
        full_redraw(now, true);
        return;
    default:
        break;
    }

    if (c >= 0x20 && c < 0x7F && lineLen < LINE_SIZE - 1) {
        line[lineLen++] = (char)c;
        line[lineLen] = '\0';
        ui_set_prompt(line, lineLen);
    }
}

void terminal_poll(uint32_t now)
{
    if (redrawPending && reached(now, redrawAt)) {
        redrawPending = false;
        inputState = IN_NORMAL;
        full_redraw(now, true);
    }

    if (inputState == IN_ESC && reached(now, escTime + ESC_TIMEOUT_MS)) {
        inputState = IN_NORMAL;
        send_stop_key(now);
    }

    struct LogMsg msg;
    while (log_receive(&msg))
        ui_log(&msg);

    if (reached(now, nextUiUpdate)) {
        nextUiUpdate = now + UI_PERIOD_MS;
        refresh_data(now, false);
        ui_update(&data);
    }
}

uint32_t terminal_wait_time(uint32_t now)
{
    int32_t wait = (int32_t)(nextUiUpdate - now);

    if (inputState == IN_ESC) {
        int32_t esc = (int32_t)(escTime + ESC_TIMEOUT_MS - now);
        if (esc < wait)
            wait = esc;
    }
    if (redrawPending) {
        int32_t redraw = (int32_t)(redrawAt - now);
        if (redraw < wait)
            wait = redraw;
    }
    return wait > 0 ? (uint32_t)wait : 0;
}

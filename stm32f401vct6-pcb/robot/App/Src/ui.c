/**
 * @file    ui.c
 * @brief   Screen layout and drawing.
 *
 * ui_redraw() draws everything once: labels, separators, every value and the
 * recent log lines. After that ui_update() formats each value again and only
 * rewrites those whose text changed: move the cursor there, write the text
 * padded to the field's width, put the cursor back at the end of the prompt.
 *
 * The log area is a VT100 scrolling region. A new line goes in at its bottom
 * and only the region scrolls, so the panel above and the prompt below stay.
 *
 * To add a value to the panel: write a fmt_ function, add it to enum FieldId
 * and fields[], and add a label to labels[] if it needs one.
 */
#include <stdio.h>
#include <string.h>

#include "ui.h"
#include "term.h"

#define CSI                 "\x1b["

#define DEFAULT_ROWS        24
#define DEFAULT_COLS        80
#define MIN_ROWS            12
#define MAX_ROWS            200
#define MIN_COLS            40
#define MAX_COLS            250

#define LOG_HISTORY         32      /* log lines kept so a redraw can show them again */
#define FIELD_TEXT_SIZE     80

/* text styles */
#define S_NONE              ""
#define S_BOLD              CSI "1m"
#define S_DIM               CSI "2m"
#define S_RED               CSI "1;31m"
#define S_GREEN             CSI "1;32m"
#define S_YELLOW            CSI "1;33m"
#define S_CYAN              CSI "36m"
#define S_MAGENTA           CSI "1;35m"
#define S_ALERT             CSI "1;37;41m"     /* white on red */
#define S_RESET             CSI "0m"

/* screen rows */
#define ROW_TITLE           1
#define ROW_SEP1            2
#define ROW_MODE            3
#define ROW_DRILL           4
#define ROW_ARM             5
#define ROW_ROTATION        6
#define ROW_SEP2            7
#define ROW_SERVO           8       /* debug rows */
#define ROW_HEAP            9
#define ROW_STACKS          10
#define ROW_SEP3            11
#define LOG_TOP             8       /* first log row, normal layout */
#define LOG_TOP_DEBUG       12      /* first log row, debug layout */

typedef void (*FormatFn)(const struct UiData *d, char *text, size_t size, const char **style);

struct Field {
    uint8_t row;
    uint8_t col;
    uint8_t width;
    bool debugOnly;
    FormatFn format;
};

struct Label {
    uint8_t row;
    uint8_t col;
    bool debugOnly;
    const char *style;
    const char *text;
};

/* ------------------------------------------------------------------------- */
/* Formatting values                                                          */
/* ------------------------------------------------------------------------- */

/* tenths -> "+2.5" / "-0.3" */
static void format_tenths(char *buf, size_t size, int32_t tenths)
{
    int32_t a = tenths < 0 ? -tenths : tenths;
    snprintf(buf, size, "%c%ld.%ld", tenths < 0 ? '-' : '+', (long)(a / 10), (long)(a % 10));
}

static const char *step_name(enum StepAction a)
{
    switch (a) {
    case STEP_RETRACT:   return "retract";
    case STEP_DRILL_ON:  return "drill on";
    case STEP_EXTEND:    return "extend";
    case STEP_DRILL_OFF: return "drill off";
    case STEP_NONE:      break;
    }
    return "-";
}

static void fmt_sim(const struct UiData *d, char *t, size_t n, const char **style)
{
    if (d->state.simulated) {
        snprintf(t, n, "SIMULATED");
        *style = S_MAGENTA;
    } else {
        t[0] = '\0';
    }
}

static void fmt_uptime(const struct UiData *d, char *t, size_t n, const char **style)
{
    (void)style;
    unsigned long s = d->now / 1000;
    snprintf(t, n, "up %02lu:%02lu:%02lu", s / 3600, (s / 60) % 60, s % 60);
}

static void fmt_mode(const struct UiData *d, char *t, size_t n, const char **style)
{
    switch (d->state.mode) {
    case MODE_IDLE:    snprintf(t, n, "IDLE");                          break;
    case MODE_AUTO:    snprintf(t, n, "AUTO");    *style = S_GREEN;     break;
    case MODE_MANUAL:  snprintf(t, n, "MANUAL");  *style = S_YELLOW;    break;
    case MODE_STOPPED: snprintf(t, n, "STOPPED"); *style = S_RED;       break;
    case MODE_FAULT:   snprintf(t, n, "FAULT");   *style = S_ALERT;     break;
    default:           snprintf(t, n, "?");                             break;
    }
}

static void fmt_step(const struct UiData *d, char *t, size_t n, const char **style)
{
    (void)style;
    const struct State *s = &d->state;
    if (s->mode == MODE_IDLE || s->seqStepCount == 0)
        snprintf(t, n, "-");
    else
        snprintf(t, n, "%u/%u %s", s->seqStep + 1u, s->seqStepCount, step_name(s->seqAction));
}

static void fmt_safety(const struct UiData *d, char *t, size_t n, const char **style)
{
    snprintf(t, n, d->state.safetyEnabled ? "ON" : "OFF");
    *style = d->state.safetyEnabled ? S_GREEN : S_ALERT;
}

static void fmt_debug(const struct UiData *d, char *t, size_t n, const char **style)
{
    snprintf(t, n, d->state.debugEnabled ? "on" : "off");
    if (d->state.debugEnabled)
        *style = S_YELLOW;
}

static void fmt_drill(const struct UiData *d, char *t, size_t n, const char **style)
{
    snprintf(t, n, "%-3s  %3u%%", d->state.drillOn ? "on" : "off", d->state.drillSpeed);
    if (d->state.drillOn)
        *style = S_YELLOW;
}

static void fmt_arm(const struct UiData *d, char *t, size_t n, const char **style)
{
    const char *motion = "stopped";
    bool pulsing = d->state.drivePulsing && d->state.driveMotion != DRIVE_STOPPED;
    if (d->state.driveMotion == DRIVE_EXTENDING)
        motion = pulsing ? "pulse out" : "extending";
    else if (d->state.driveMotion == DRIVE_RETRACTING)
        motion = pulsing ? "pulse in" : "retracting";
    snprintf(t, n, "%-10s %3u%%", motion, pulsing ? 100u : d->state.driveSpeed);
    if (d->state.driveMotion != DRIVE_STOPPED)
        *style = S_CYAN;
}

static void fmt_timeout(const struct UiData *d, char *t, size_t n, const char **style)
{
    (void)style;
    uint32_t ms = d->state.driveTimeoutMs;
    snprintf(t, n, "%lu.%lu s", (unsigned long)(ms / 1000u), (unsigned long)(ms % 1000u / 100u));
}

static void fmt_limits(const struct UiData *d, char *t, size_t n, const char **style)
{
    (void)style;
    snprintf(t, n, "min [%c]  max [%c]",
             d->state.atExtMin ? 'x' : ' ', d->state.atExtMax ? 'x' : ' ');
}

static void fmt_angle(const struct UiData *d, char *t, size_t n, const char **style)
{
    if (!d->state.servoOnline) {
        snprintf(t, n, "--");
        *style = S_DIM;
        return;
    }
    char a[12];
    format_tenths(a, sizeof a, d->state.rotateAngle);
    snprintf(t, n, "%s deg", a);
}

static void fmt_target(const struct UiData *d, char *t, size_t n, const char **style)
{
    (void)style;
    char a[12];
    format_tenths(a, sizeof a, d->state.rotateTarget);
    snprintf(t, n, "target %s deg  speed %3u%%", a, d->state.rotateSpeed);
}

static void fmt_rotation_state(const struct UiData *d, char *t, size_t n, const char **style)
{
    if (!d->state.servoOnline) {
        snprintf(t, n, "offline");
        *style = S_RED;
    } else if (d->state.servoFree) {
        snprintf(t, n, "free");
        *style = S_YELLOW;
    } else if (d->state.rotating) {
        snprintf(t, n, "moving");
        *style = S_CYAN;
    } else {
        snprintf(t, n, "holding");
    }
}

static void fmt_servo(const struct UiData *d, char *t, size_t n, const char **style)
{
    const struct State *s = &d->state;
    if (!s->servoOnline) {
        snprintf(t, n, "not responding");
        *style = S_RED;
        return;
    }
    char load[12], lo[12], hi[12];
    format_tenths(load, sizeof load, s->servoLoad);
    format_tenths(lo, sizeof lo, s->servoLimitMin);
    format_tenths(hi, sizeof hi, s->servoLimitMax);
    int len = snprintf(t, n, "%u.%u V   %d C   load %s %%   limits %s to %s deg",
                       s->servoVoltage / 10u, s->servoVoltage % 10u, s->servoTemperature, load, lo, hi);
    if (s->servoError != 0 && len > 0 && (size_t)len < n) {
        snprintf(t + len, n - (size_t)len, "   ERROR 0x%02x", s->servoError);
        *style = S_RED;
    }
}

static void fmt_heap(const struct UiData *d, char *t, size_t n, const char **style)
{
    const struct SysInfo *sys = &d->sys;
    int len = snprintf(t, n, "%lu bytes free, lowest %lu",
                       (unsigned long)sys->heapFree, (unsigned long)sys->heapMinFree);
    if (sys->logDropped > 0 && len > 0 && (size_t)len < n) {
        snprintf(t + len, n - (size_t)len, "   log lines dropped: %lu", (unsigned long)sys->logDropped);
        *style = S_YELLOW;
    }
}

static void fmt_stacks(const struct UiData *d, char *t, size_t n, const char **style)
{
    (void)style;
    size_t used = 0;
    t[0] = '\0';
    for (unsigned i = 0; i < d->sys.taskCount && used + 1 < n; i++) {
        int len = snprintf(t + used, n - used, "%s%s:%lu", i ? " " : "",
                           d->sys.tasks[i].name, (unsigned long)d->sys.tasks[i].stackFree);
        if (len < 0)
            break;
        used += (size_t)len;
    }
}

/* ------------------------------------------------------------------------- */
/* Layout                                                                     */
/* ------------------------------------------------------------------------- */

enum FieldId {
    F_SIM, F_UPTIME,
    F_MODE, F_STEP, F_SAFETY,
    F_DRILL, F_DEBUG,
    F_ARM, F_LIMITS, F_TIMEOUT,
    F_ANGLE, F_TARGET, F_ROTATION_STATE,
    F_SERVO, F_HEAP, F_STACKS,
    FIELD_COUNT
};

static const struct Field fields[FIELD_COUNT] = {
    /*                     row           col width debug  format */
    [F_SIM]            = { ROW_TITLE,     40, 10, false, fmt_sim },
    [F_UPTIME]         = { ROW_TITLE,     68, 12, false, fmt_uptime },
    [F_MODE]           = { ROW_MODE,      12, 10, false, fmt_mode },
    [F_STEP]           = { ROW_MODE,      30, 21, false, fmt_step },
    [F_SAFETY]         = { ROW_MODE,      61,  3, false, fmt_safety },
    [F_DRILL]          = { ROW_DRILL,     12, 10, false, fmt_drill },
    [F_DEBUG]          = { ROW_DRILL,     61,  3, false, fmt_debug },
    [F_ARM]            = { ROW_ARM,       12, 15, false, fmt_arm },
    [F_LIMITS]         = { ROW_ARM,       30, 18, false, fmt_limits },
    [F_TIMEOUT]        = { ROW_ARM,       61,  8, false, fmt_timeout },
    [F_ANGLE]          = { ROW_ROTATION,  12, 10, false, fmt_angle },
    [F_TARGET]         = { ROW_ROTATION,  24, 28, false, fmt_target },
    [F_ROTATION_STATE] = { ROW_ROTATION,  53, 10, false, fmt_rotation_state },
    [F_SERVO]          = { ROW_SERVO,     12, 67, true,  fmt_servo },
    [F_HEAP]           = { ROW_HEAP,      12, 67, true,  fmt_heap },
    [F_STACKS]         = { ROW_STACKS,    12, 67, true,  fmt_stacks },
};

static const struct Label labels[] = {
    { ROW_TITLE,     2, false, S_BOLD, "BORING ROBOT" },
    { ROW_TITLE,    16, false, S_DIM,  "METR4810 Group 7" },
    { ROW_MODE,      2, false, S_NONE, "Mode" },
    { ROW_MODE,     24, false, S_NONE, "Step" },
    { ROW_MODE,     53, false, S_NONE, "Safety" },
    { ROW_DRILL,     2, false, S_NONE, "Drill" },
    { ROW_DRILL,    53, false, S_NONE, "Debug" },
    { ROW_ARM,       2, false, S_NONE, "Arm" },
    { ROW_ARM,      53, false, S_NONE, "Timeout" },
    { ROW_ROTATION,  2, false, S_NONE, "Rotation" },
    { ROW_SERVO,     2, true,  S_NONE, "Servo" },
    { ROW_HEAP,      2, true,  S_NONE, "Heap" },
    { ROW_STACKS,    2, true,  S_NONE, "Stack" },
};

/* ------------------------------------------------------------------------- */
/* Screen state                                                               */
/* ------------------------------------------------------------------------- */

static uint16_t rows = DEFAULT_ROWS;
static uint16_t cols = DEFAULT_COLS;
static bool ready;              /* screen has been drawn; safe to update */
static bool debugLayout;        /* the drawn layout has the debug rows */
static bool hasLogArea;
static uint16_t logTop, logBottom;

static char drawnText[FIELD_COUNT][FIELD_TEXT_SIZE];
static char drawnStyle[FIELD_COUNT][16];
static bool drawnValid[FIELD_COUNT];

static const char *promptLine = "";
static size_t promptLen;

static struct LogMsg history[LOG_HISTORY];
static size_t historyNext;      /* where the next line goes */
static size_t historyCount;

/* ------------------------------------------------------------------------- */
/* Drawing helpers                                                            */
/* ------------------------------------------------------------------------- */

static void move_to(unsigned row, unsigned col)
{
    term_printf(CSI "%u;%uH", row, col);
}

static void repeat_char(char c, size_t count)
{
    char chunk[16];
    memset(chunk, c, sizeof chunk);
    while (count > 0) {
        size_t k = count < sizeof chunk ? count : sizeof chunk;
        term_write(chunk, k);
        count -= k;
    }
}

/* Writes text at (row, col), cut to width and padded with spaces to width.
 * Nothing is written past the right edge of the screen. */
static void draw_at(unsigned row, unsigned col, size_t width, const char *text, const char *style)
{
    if (row > rows || col > cols)
        return;
    size_t room = (size_t)cols - col + 1;
    if (width > room)
        width = room;
    size_t len = strlen(text);
    if (len > width)
        len = width;

    move_to(row, col);
    if (style[0] != '\0')
        term_puts(style);
    term_write(text, len);
    if (style[0] != '\0')
        term_puts(S_RESET);
    repeat_char(' ', width - len);
}

static void draw_separator(unsigned row)
{
    move_to(row, 1);
    term_puts(S_DIM);
    repeat_char('-', cols);
    term_puts(S_RESET);
}

static void park_cursor(void)
{
    move_to(rows, 3 + (unsigned)promptLen);
}

static void draw_prompt(void)
{
    move_to(rows, 1);
    term_puts(S_BOLD "> " S_RESET);
    term_write(promptLine, promptLen);
    term_puts(CSI "K");
}

/* Draws the fields that changed (or all of them). Returns true if any were. */
static bool draw_fields(const struct UiData *d, bool all)
{
    bool any = false;

    for (int i = 0; i < FIELD_COUNT; i++) {
        const struct Field *f = &fields[i];
        if (f->debugOnly && !debugLayout)
            continue;

        char text[FIELD_TEXT_SIZE];
        const char *style = S_NONE;
        text[0] = '\0';
        f->format(d, text, sizeof text, &style);

        if (!all && drawnValid[i] && strcmp(text, drawnText[i]) == 0
                 && strcmp(style, drawnStyle[i]) == 0)
            continue;

        if (!any) {
            term_puts(CSI "?25l");          /* hide the cursor while it jumps around */
            any = true;
        }
        draw_at(f->row, f->col, f->width, text, style);
        snprintf(drawnText[i], sizeof drawnText[i], "%s", text);
        snprintf(drawnStyle[i], sizeof drawnStyle[i], "%s", style);
        drawnValid[i] = true;
    }
    return any;
}

static const struct LogMsg *history_get(size_t i)  /* 0 = oldest */
{
    return &history[(historyNext + LOG_HISTORY - historyCount + i) % LOG_HISTORY];
}

/* One log line at the cursor: "hh:mm:ss text", cut to the screen width. */
static void draw_log_line(const struct LogMsg *m)
{
    unsigned long s = m->time / 1000;
    term_printf(S_DIM "%02lu:%02lu:%02lu " S_RESET, (s / 3600) % 100, (s / 60) % 60, s % 60);

    const char *style = S_NONE;
    const char *tag = "";
    switch (m->level) {
    case LOG_DEBUG: style = S_DIM;                     break;
    case LOG_INFO:                                     break;
    case LOG_WARN:  style = S_YELLOW; tag = "WARN ";   break;
    case LOG_ERROR: style = S_RED;    tag = "ERROR ";  break;
    case LOG_INPUT: style = S_BOLD;                    break;
    }

    /* keep off the last column so the terminal never wraps the line */
    size_t room = cols > 10 ? (size_t)cols - 10 : 0;
    size_t tagLen = strlen(tag);
    if (tagLen > room)
        tagLen = room;
    size_t len = strnlen(m->text, sizeof m->text);
    if (len > room - tagLen)
        len = room - tagLen;

    if (style[0] != '\0')
        term_puts(style);
    term_write(tag, tagLen);
    term_write(m->text, len);
    if (style[0] != '\0')
        term_puts(S_RESET);
    term_puts(CSI "K");
}

/* ------------------------------------------------------------------------- */
/* Public                                                                     */
/* ------------------------------------------------------------------------- */

void ui_init(void)
{
    rows = DEFAULT_ROWS;
    cols = DEFAULT_COLS;
    ready = false;
    historyNext = 0;
    historyCount = 0;
    promptLine = "";
    promptLen = 0;
}

void ui_invalidate(void)
{
    ready = false;
}

bool ui_set_size(uint16_t newRows, uint16_t newCols)
{
    if (newRows < MIN_ROWS) newRows = MIN_ROWS;
    if (newRows > MAX_ROWS) newRows = MAX_ROWS;
    if (newCols < MIN_COLS) newCols = MIN_COLS;
    if (newCols > MAX_COLS) newCols = MAX_COLS;
    if (newRows == rows && newCols == cols)
        return false;
    rows = newRows;
    cols = newCols;
    return true;
}

void ui_query_size(void)
{
    /* The cursor can't go past the bottom-right corner, so after this jump
     * its position is the screen size. "ESC [ 6 n" asks the terminal to
     * report the position. */
    term_puts(CSI "999;999H" CSI "6n");
    park_cursor();
}

void ui_redraw(const struct UiData *d)
{
    debugLayout = d->state.debugEnabled;
    logTop = debugLayout ? LOG_TOP_DEBUG : LOG_TOP;
    logBottom = rows - 1;
    hasLogArea = logBottom > logTop;    /* a scrolling region needs 2+ rows */

    /* hide cursor, normal text, whole screen scrolls, clear */
    term_puts(CSI "?25l" S_RESET CSI "r" CSI "2J");

    for (size_t i = 0; i < sizeof labels / sizeof labels[0]; i++) {
        const struct Label *l = &labels[i];
        if (!l->debugOnly || debugLayout)
            draw_at(l->row, l->col, strlen(l->text), l->text, l->style);
    }
    draw_separator(ROW_SEP1);
    draw_separator(ROW_SEP2);
    if (debugLayout)
        draw_separator(ROW_SEP3);

    draw_fields(d, true);

    if (hasLogArea) {
        term_printf(CSI "%u;%ur", logTop, logBottom);  /* the scrolling region */
        size_t height = (size_t)logBottom - logTop + 1;
        size_t shown = historyCount < height ? historyCount : height;
        for (size_t i = 0; i < shown; i++) {
            move_to(logBottom - (unsigned)shown + 1 + (unsigned)i, 1);
            draw_log_line(history_get(historyCount - shown + i));
        }
    }

    ready = true;
    draw_prompt();
    term_puts(CSI "?25h");
}

void ui_update(const struct UiData *d)
{
    if (!ready)
        return;
    if (d->state.debugEnabled != debugLayout) {
        ui_redraw(d);                   /* debug rows appear or go */
        return;
    }
    if (draw_fields(d, false)) {
        park_cursor();
        term_puts(CSI "?25h");
    }
}

void ui_log(const struct LogMsg *msg)
{
    history[historyNext] = *msg;
    historyNext = (historyNext + 1) % LOG_HISTORY;
    if (historyCount < LOG_HISTORY)
        historyCount++;

    if (!ready || !hasLogArea)
        return;
    term_puts(CSI "?25l");
    move_to(logBottom, 1);
    term_puts("\n\r");                  /* at the region's bottom, LF scrolls just the region */
    draw_log_line(msg);
    park_cursor();
    term_puts(CSI "?25h");
}

void ui_log_clear(void)
{
    historyCount = 0;
    if (!ready || !hasLogArea)
        return;
    for (unsigned r = logTop; r <= logBottom; r++) {
        move_to(r, 1);
        term_puts(CSI "K");
    }
    park_cursor();
}

void ui_set_prompt(const char *line, size_t len)
{
    promptLine = line;
    promptLen = len;
    if (ready)
        draw_prompt();
}

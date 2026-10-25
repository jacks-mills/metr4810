/**
 * @file    console.c
 * @brief   The command language: typed text <-> struct Command.
 *
 * A command is one or two words, then any numbers:
 *     drill speed 60      rotate to -1.5      servo limits -2.5 2.5
 *     drive timeout 12.5  pulse extend 200
 * Case doesn't matter. Angles (degrees) and times (seconds) take at most one
 * decimal place.
 *
 * To add a command: add it to enum CommandType (command.h), to syntax[] and
 * commandNames[] below, and to the help text.
 */
#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "console.h"

#define MAX_WORDS   6
#define MAX_LINE    80

/* drive timeout range, tenths of a second */
#define TIMEOUT_MIN 10          /* 1 s */
#define TIMEOUT_MAX 6000        /* 10 min */

/* pulse length range, ms */
#define PULSE_MIN   1
#define PULSE_MAX   10000

enum ArgKind {
    ARG_NONE,           /* no numbers */
    ARG_FIXED,          /* no numbers; value comes from the table (on/off) */
    ARG_PERCENT,        /* one number 0-100 */
    ARG_ANGLE,          /* one angle */
    ARG_ANGLE_RANGE,    /* two angles, min then max */
    ARG_SECONDS,        /* one time in seconds; value in ms */
    ARG_MS              /* one whole number of ms (a pulse length) */
};

struct Syntax {
    const char *word1;
    const char *word2;          /* NULL for one-word commands */
    enum CommandType type;
    enum ArgKind arg;
    int32_t fixedValue;
};

static const struct Syntax syntax[] = {
    { "start",   NULL,      CMD_SEQ_START,        ARG_NONE,        0 },
    { "restart", NULL,      CMD_SEQ_RESTART,      ARG_NONE,        0 },
    { "pause",   NULL,      CMD_SEQ_PAUSE,        ARG_NONE,        0 },
    { "stop",    NULL,      CMD_SEQ_STOP,         ARG_NONE,        0 },

    { "safety",  "on",      CMD_SAFETY_SET,       ARG_FIXED,       1 },
    { "safety",  "off",     CMD_SAFETY_SET,       ARG_FIXED,       0 },
    { "debug",   "on",      CMD_DEBUG_SET,        ARG_FIXED,       1 },
    { "debug",   "off",     CMD_DEBUG_SET,        ARG_FIXED,       0 },

    { "drill",   "start",   CMD_DRILL_START,      ARG_NONE,        0 },
    { "drill",   "stop",    CMD_DRILL_STOP,       ARG_NONE,        0 },
    { "drill",   "speed",   CMD_DRILL_SET_SPEED,  ARG_PERCENT,     0 },

    { "drive",   "extend",  CMD_DRIVE_EXTEND,     ARG_NONE,        0 },
    { "drive",   "retract", CMD_DRIVE_RETRACT,    ARG_NONE,        0 },
    { "drive",   "stop",    CMD_DRIVE_STOP,       ARG_NONE,        0 },
    { "drive",   "speed",   CMD_DRIVE_SET_SPEED,  ARG_PERCENT,     0 },
    { "drive",   "timeout", CMD_DRIVE_SET_TIMEOUT, ARG_SECONDS,    0 },

    { "pulse",   "extend",  CMD_DRIVE_PULSE_EXTEND,  ARG_MS,       0 },
    { "pulse",   "retract", CMD_DRIVE_PULSE_RETRACT, ARG_MS,       0 },

    { "rotate",  "left",    CMD_ROTATE_LEFT,      ARG_NONE,        0 },
    { "rotate",  "right",   CMD_ROTATE_RIGHT,     ARG_NONE,        0 },
    { "rotate",  "stop",    CMD_ROTATE_STOP,      ARG_NONE,        0 },
    { "rotate",  "to",      CMD_ROTATE_TO,        ARG_ANGLE,       0 },
    { "rotate",  "speed",   CMD_ROTATE_SET_SPEED, ARG_PERCENT,     0 },

    { "servo",   "free",    CMD_SERVO_FREE,       ARG_NONE,        0 },
    { "servo",   "hold",    CMD_SERVO_HOLD,       ARG_NONE,        0 },
    { "servo",   "centre",  CMD_SERVO_CENTRE,     ARG_NONE,        0 },
    { "servo",   "center",  CMD_SERVO_CENTRE,     ARG_NONE,        0 },
    { "servo",   "limits",  CMD_SERVO_LIMITS,     ARG_ANGLE_RANGE, 0 },
};

static const struct {
    const char *word1;
    const char *usage;
} usages[] = {
    { "safety", "usage: safety on|off" },
    { "debug",  "usage: debug on|off" },
    { "drill",  "usage: drill start|stop|speed <0-100>" },
    { "drive",  "usage: drive extend|retract|stop|speed <0-100>|timeout <s>" },
    { "pulse",  "usage: pulse extend|retract <ms>   (full power, then stop)" },
    { "rotate", "usage: rotate left|right|stop|to <deg>|speed <0-100>" },
    { "servo",  "usage: servo free|hold|centre|limits <min deg> <max deg>" },
};

static const char *const commandNames[CMD_COUNT] = {
    [CMD_NONE]             = "none",
    [CMD_SEQ_START]        = "start",
    [CMD_SEQ_RESTART]      = "restart",
    [CMD_SEQ_PAUSE]        = "pause",
    [CMD_SEQ_STOP]         = "stop",
    [CMD_SAFETY_SET]       = "safety",
    [CMD_DEBUG_SET]        = "debug",
    [CMD_DRILL_START]      = "drill start",
    [CMD_DRILL_STOP]       = "drill stop",
    [CMD_DRILL_SET_SPEED]  = "drill speed",
    [CMD_DRIVE_EXTEND]     = "drive extend",
    [CMD_DRIVE_RETRACT]    = "drive retract",
    [CMD_DRIVE_STOP]       = "drive stop",
    [CMD_DRIVE_SET_SPEED]  = "drive speed",
    [CMD_DRIVE_SET_TIMEOUT] = "drive timeout",
    [CMD_DRIVE_PULSE_EXTEND] = "pulse extend",
    [CMD_DRIVE_PULSE_RETRACT] = "pulse retract",
    [CMD_ROTATE_LEFT]      = "rotate left",
    [CMD_ROTATE_RIGHT]     = "rotate right",
    [CMD_ROTATE_STOP]      = "rotate stop",
    [CMD_ROTATE_TO]        = "rotate to",
    [CMD_ROTATE_SET_SPEED] = "rotate speed",
    [CMD_SERVO_FREE]       = "servo free",
    [CMD_SERVO_HOLD]       = "servo hold",
    [CMD_SERVO_CENTRE]     = "servo centre",
    [CMD_SERVO_LIMITS]     = "servo limits",
    [CMD_TERMINAL_LOST]    = "terminal closed",
    [EVT_DRIVE_RESULT]     = "drive result",
    [EVT_ROTATE_RESULT]    = "rotate result",
};

const char *const console_help_lines[] = {
    "Commands:",
    "  start | restart | pause | stop     sequence (Esc also stops)",
    "  safety on|off     debug on|off",
    "  drill start|stop|speed <0-100>",
    "  drive extend|retract|stop|speed <0-100>|timeout <s>",
    "  pulse extend|retract <ms>    arm at full power for <ms>, then stop",
    "  rotate left|right|stop|to <deg>|speed <0-100>",
    "  servo free|hold|centre|limits <min> <max>   (debug mode)",
    "  clear | redraw | help     Ctrl-L redraws, Up/Down recall",
    NULL
};

const char *console_command_name(enum CommandType type)
{
    if ((unsigned)type < CMD_COUNT && commandNames[type] != NULL)
        return commandNames[type];
    return "?";
}

/* Splits buf in place into lower-case words. Returns the number of words, or
 * MAX_WORDS + 1 if there were too many. */
static size_t split_words(char *buf, char *words[])
{
    size_t count = 0;
    char *p = buf;

    for (;;) {
        while (*p == ' ' || *p == '\t')
            p++;
        if (*p == '\0')
            return count;
        if (count == MAX_WORDS)
            return MAX_WORDS + 1;
        words[count++] = p;
        while (*p != '\0' && *p != ' ' && *p != '\t') {
            *p = (char)tolower((unsigned char)*p);
            p++;
        }
        if (*p != '\0')
            *p++ = '\0';
    }
}

/* Whole number in [min, max]. */
static bool parse_int(const char *s, int32_t min, int32_t max, int32_t *out)
{
    char *end;
    long v = strtol(s, &end, 10);

    if (end == s || *end != '\0' || v < min || v > max)
        return false;
    *out = (int32_t)v;
    return true;
}

/* A number with at most one decimal place -> tenths: "-2.5" -> -25. Used for
 * degrees and seconds. */
static bool parse_tenths(const char *s, int32_t *out)
{
    bool negative = false;
    int32_t tenths = 0;
    int digits = 0;

    if (*s == '+' || *s == '-')
        negative = (*s++ == '-');
    while (isdigit((unsigned char)*s)) {
        tenths = tenths * 10 + (*s++ - '0');
        if (++digits > 3)
            return false;               /* 1000 or more */
    }
    tenths *= 10;
    if (*s == '.') {
        s++;
        if (isdigit((unsigned char)*s)) {
            tenths += *s++ - '0';
            digits++;
        }
    }
    if (digits == 0 || *s != '\0')
        return false;
    *out = negative ? -tenths : tenths;
    return true;
}

static void set_usage(const char *word1, char *error, size_t errorSize)
{
    for (size_t i = 0; i < sizeof usages / sizeof usages[0]; i++) {
        if (strcmp(usages[i].word1, word1) == 0) {
            snprintf(error, errorSize, "%s", usages[i].usage);
            return;
        }
    }
    snprintf(error, errorSize, "'%s' takes nothing after it", word1);
}

enum ConsoleResult console_parse(const char *line, struct Command *cmd,
                                 char *error, size_t errorSize)
{
    char buf[MAX_LINE + 1];
    char *words[MAX_WORDS];

    snprintf(buf, sizeof buf, "%s", line);
    size_t count = split_words(buf, words);

    if (count == 0)
        return CONSOLE_EMPTY;
    if (count > MAX_WORDS) {
        snprintf(error, errorSize, "too many words");
        return CONSOLE_ERROR;
    }

    /* commands handled by the terminal itself */
    if (count == 1) {
        if (strcmp(words[0], "help") == 0 || strcmp(words[0], "?") == 0)
            return CONSOLE_HELP;
        if (strcmp(words[0], "clear") == 0)
            return CONSOLE_CLEAR;
        if (strcmp(words[0], "redraw") == 0)
            return CONSOLE_REDRAW;
    }

    const struct Syntax *match = NULL;
    bool knownFirstWord = false;

    for (size_t i = 0; i < sizeof syntax / sizeof syntax[0]; i++) {
        const struct Syntax *s = &syntax[i];
        if (strcmp(s->word1, words[0]) != 0)
            continue;
        knownFirstWord = true;
        if (s->word2 == NULL || (count >= 2 && strcmp(s->word2, words[1]) == 0)) {
            match = s;
            break;
        }
    }

    if (!knownFirstWord) {
        snprintf(error, errorSize, "unknown command '%.20s' (type help)", words[0]);
        return CONSOLE_ERROR;
    }
    if (match == NULL) {
        set_usage(words[0], error, errorSize);
        return CONSOLE_ERROR;
    }

    size_t first = (match->word2 != NULL) ? 2 : 1;      /* index of first number */
    size_t numbers = count - first;
    size_t wanted = (match->arg == ARG_PERCENT || match->arg == ARG_ANGLE ||
                     match->arg == ARG_SECONDS || match->arg == ARG_MS) ? 1
                  : (match->arg == ARG_ANGLE_RANGE) ? 2 : 0;

    if (numbers != wanted) {
        set_usage(words[0], error, errorSize);
        return CONSOLE_ERROR;
    }

    cmd->type = match->type;
    cmd->source = SOURCE_CONSOLE;
    cmd->value = match->fixedValue;
    cmd->value2 = 0;

    switch (match->arg) {
    case ARG_PERCENT:
        if (!parse_int(words[first], 0, 100, &cmd->value)) {
            snprintf(error, errorSize, "speed must be a whole number 0-100");
            return CONSOLE_ERROR;
        }
        break;
    case ARG_ANGLE:
        if (!parse_tenths(words[first], &cmd->value)) {
            snprintf(error, errorSize, "angle must be in degrees, like 1.5 or -2");
            return CONSOLE_ERROR;
        }
        break;
    case ARG_ANGLE_RANGE:
        if (!parse_tenths(words[first], &cmd->value) ||
            !parse_tenths(words[first + 1], &cmd->value2)) {
            snprintf(error, errorSize, "angles must be in degrees, like -2.5 2.5");
            return CONSOLE_ERROR;
        }
        if (cmd->value >= cmd->value2) {
            snprintf(error, errorSize, "min must be less than max");
            return CONSOLE_ERROR;
        }
        break;
    case ARG_SECONDS:
        if (!parse_tenths(words[first], &cmd->value) ||
            cmd->value < TIMEOUT_MIN || cmd->value > TIMEOUT_MAX) {
            snprintf(error, errorSize, "timeout must be %d-%d seconds, like 20 or 12.5",
                     TIMEOUT_MIN / 10, TIMEOUT_MAX / 10);
            return CONSOLE_ERROR;
        }
        cmd->value *= 100;              /* tenths of a second -> ms */
        break;
    case ARG_MS:
        if (!parse_int(words[first], PULSE_MIN, PULSE_MAX, &cmd->value)) {
            snprintf(error, errorSize, "pulse must be a whole number of ms, %d-%d",
                     PULSE_MIN, PULSE_MAX);
            return CONSOLE_ERROR;
        }
        break;
    case ARG_NONE:
    case ARG_FIXED:
        break;
    }
    return CONSOLE_COMMAND;
}

/**
 * @file    console.h
 * @brief   The command language: typed text <-> struct Command.
 *
 * Pure text handling, no RTOS calls, so it can be tested on a PC.
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stddef.h>
#include "command.h"

enum ConsoleResult {
    CONSOLE_EMPTY,      /* blank line */
    CONSOLE_COMMAND,    /* *cmd is filled in */
    CONSOLE_HELP,       /* "help": show console_help_lines */
    CONSOLE_CLEAR,      /* "clear": clear the log area */
    CONSOLE_REDRAW,     /* "redraw": redraw the screen */
    CONSOLE_ERROR       /* error holds a message for the user */
};

enum ConsoleResult console_parse(const char *line, struct Command *cmd,
                                 char *error, size_t errorSize);

/* The words that produce a command type, e.g. "drill speed". */
const char *console_command_name(enum CommandType type);

/* Help text, one string per line, ending with NULL. */
extern const char *const console_help_lines[];

#endif /* CONSOLE_H */

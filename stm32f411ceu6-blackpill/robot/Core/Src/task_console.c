/**
 * @file    task_console.c
 * @brief   console task: reads commands from USB serial and updates state.
 *
 * This task is set to "As external" in CubeMX, so CubeMX only creates the
 * thread; the function body lives here and regenerating won't touch it.
 *
 * It's the only task that reads stdin. fgets blocks (inside _read) until a
 * line arrives, so the task uses no CPU while waiting.
 */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "main.h"
#include "cmsis_os.h"
#include "robot_state.h"
#include "task_console.h"

static void print_help(void)
{
    printf("commands:\r\n");
    printf("  help                show this list\r\n");
    printf("  auto | pause | stop set mode\r\n");
    printf("  safety on|off\r\n");
    printf("  debug on|off\r\n");
    printf("  drill <speed>\r\n");
    printf("  drive fwd|rev <speed>\r\n");
    printf("  rotate <angle>\r\n");
}

/* "on" -> true, "off" -> false. Returns false if it's neither. */
static bool parse_on_off(const char *word, bool *out)
{
    if (strcmp(word, "on") == 0)  { *out = true;  return true; }
    if (strcmp(word, "off") == 0) { *out = false; return true; }
    return false;
}

/* Runs one command line. Returns true if the state changed. */
static bool handle_command(struct State *s, const char *line)
{
    char word[16];
    char arg[16];
    unsigned long value;
    long angle;
    bool flag;

    if (strcmp(line, "help") == 0) {
        print_help();
        return false;
    }
    if (strcmp(line, "auto") == 0)  { s->mode = AUTO;    return true; }
    if (strcmp(line, "pause") == 0) { s->mode = PAUSED;  return true; }
    if (strcmp(line, "stop") == 0)  { s->mode = STOPPED; return true; }

    if (sscanf(line, "safety %15s", arg) == 1 && parse_on_off(arg, &flag)) {
        s->safetyIsEnabled = flag;
        return true;
    }
    if (sscanf(line, "debug %15s", arg) == 1 && parse_on_off(arg, &flag)) {
        s->debugIsEnabled = flag;
        return true;
    }
    if (sscanf(line, "drill %lu", &value) == 1) {
        s->drillSpeed = value;
        return true;
    }
    if (sscanf(line, "drive %15s %lu", arg, &value) == 2) {
        if (strcmp(arg, "fwd") == 0) {
            s->driveDirection = FORWARD;
        } else if (strcmp(arg, "rev") == 0) {
            s->driveDirection = REVERSE;
        } else {
            printf("drive direction must be fwd or rev\r\n");
            return false;
        }
        s->driveSpeed = value;
        return true;
    }
    if (sscanf(line, "rotate %ld", &angle) == 1) {
        s->rotationAngleSet = angle;
        return true;
    }

    sscanf(line, "%15s", word);
    printf("unknown command: %s (type 'help')\r\n", word);
    return false;
}

void TaskConsole(void *argument)
{
    struct Params *params = (struct Params *)argument;
    char line[64];

    /* Unbuffered stdin: newlib skips allocating its 1 KB stdin buffer and
     * fetches input straight from _read. fgets behaves the same. */
    setvbuf(stdin, NULL, _IONBF, 0);

    for (;;)
    {
        printf("> ");
        fflush(stdout);                     /* prompt has no '\n', so push it out */

        if (fgets(line, sizeof line, stdin) == NULL)
            continue;

        line[strcspn(line, "\n")] = '\0';   /* strip the newline */
        if (line[0] == '\0')
            continue;                       /* empty line */

        if (handle_command(params->state, line))
            (void)osSemaphoreRelease(params->updateUISemHandle);
    }
}

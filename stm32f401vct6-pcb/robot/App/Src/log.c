/**
 * @file    log.c
 * @brief   Log messages for the terminal.
 */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"
#include "log.h"
#include "robot_state.h"

#define LOG_QUEUE_LENGTH    16

static osMessageQueueId_t queue;
static volatile uint32_t dropped;

void log_init(void)
{
    queue = osMessageQueueNew(LOG_QUEUE_LENGTH, sizeof(struct LogMsg),
                              &(osMessageQueueAttr_t){ .name = "log" });
}

static void log_vmsg(enum LogLevel level, const char *fmt, va_list args)
{
    if (level == LOG_DEBUG && !state.debugEnabled)
        return;

    struct LogMsg msg;
    msg.time = osKernelGetTickCount();
    msg.level = level;
    vsnprintf(msg.text, sizeof msg.text, fmt, args);

    if (queue == NULL || osMessageQueuePut(queue, &msg, 0, 0) != osOK)
        dropped++;
}

void log_msg(enum LogLevel level, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    log_vmsg(level, fmt, args);
    va_end(args);
}

bool log_receive(struct LogMsg *msg)
{
    return queue != NULL && osMessageQueueGet(queue, msg, NULL, 0) == osOK;
}

uint32_t log_dropped(void)
{
    return dropped;
}

/* printf() and puts() end up here. Their output is turned into log lines, so
 * a stray printf can't scramble the screen. Use log_info() etc. instead where
 * you can: printing to stdout makes newlib allocate a buffer from the C heap,
 * which isn't protected against two tasks doing it at once. */
int _write(int file, char *ptr, int len)
{
    (void)file;
    int start = 0;

    for (int i = 0; i <= len; i++) {
        if (i == len || ptr[i] == '\n' || ptr[i] == '\r') {
            if (i > start)
                log_msg(LOG_INFO, "%.*s", i - start, ptr + start);
            start = i + 1;
        }
    }
    return len;
}

/**
 * @file    log.h
 * @brief   Log messages for the terminal.
 *
 * Any task (not the terminal task itself) reports what's happening with
 * log_info(), log_warn() etc. The text is queued and the terminal task prints
 * it in the scrolling area under the status panel, so tasks never write to
 * the terminal directly and can't scramble the screen.
 *
 * Debug-level messages are dropped unless debug mode is on.
 */
#ifndef LOG_H
#define LOG_H

#include <stdbool.h>
#include <stdint.h>

enum LogLevel {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    LOG_INPUT       /* echo of a typed command (terminal only) */
};

#define LOG_TEXT_SIZE 76

struct LogMsg {
    uint32_t time;              /* kernel ticks (ms) */
    enum LogLevel level;
    char text[LOG_TEXT_SIZE];
};

/* Creates the queue. Called from main() after osKernelInitialize(). */
void log_init(void);

/* printf-style. Never blocks; if the queue is full the message is dropped
 * and counted (shown in the debug panel). Long messages are cut short. */
void log_msg(enum LogLevel level, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

#define log_debug(...) log_msg(LOG_DEBUG, __VA_ARGS__)
#define log_info(...)  log_msg(LOG_INFO, __VA_ARGS__)
#define log_warn(...)  log_msg(LOG_WARN, __VA_ARGS__)
#define log_error(...) log_msg(LOG_ERROR, __VA_ARGS__)

/* Terminal task only: takes the next queued message, without waiting. */
bool log_receive(struct LogMsg *msg);

/* Number of messages dropped because the queue was full. */
uint32_t log_dropped(void);

#endif /* LOG_H */

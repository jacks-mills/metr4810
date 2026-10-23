/**
 * @file    task_console.h
 * @brief   console task: reads commands from USB serial and updates state.
 */
#ifndef TASK_CONSOLE_H
#define TASK_CONSOLE_H

/* Entry function for the console thread (created in main.c by CubeMX).
 * argument: struct Params *
 * This is the only task that should read stdin. */
void TaskConsole(void *argument);

#endif /* TASK_CONSOLE_H */

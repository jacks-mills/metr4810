/**
 * @file    task_default.h
 * @brief   defaultTask: brings up USB CDC, then blinks the LED.
 */
#ifndef TASK_DEFAULT_H
#define TASK_DEFAULT_H

/* Entry function for the defaultTask thread (created in main.c by CubeMX).
 * argument: struct Params * */
void TaskDefault(void *argument);

#endif /* TASK_DEFAULT_H */

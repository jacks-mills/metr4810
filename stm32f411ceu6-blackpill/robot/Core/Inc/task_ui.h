/**
 * @file    task_ui.h
 * @brief   ui task.
 */
#ifndef TASK_UI_H
#define TASK_UI_H

/* Entry function for the ui thread (created in main.c by CubeMX).
 * argument: struct Params * */
void TaskUI(void *argument);

#endif /* TASK_UI_H */

/**
 * @file    app.h
 * @brief   Entry points main.c calls (inside its USER CODE sections).
 */
#ifndef APP_H
#define APP_H

/* USER CODE 2, before MX_USB_DEVICE_Init(): things the USB interrupt needs. */
void app_init_early(void);

/* USER CODE RTOS_QUEUES (after osKernelInitialize()): queues. */
void app_init_rtos(void);

/* Task entry functions. CubeMX creates the tasks ("As external"). */
void TaskControl(void *argument);
void TaskTerminal(void *argument);

#endif /* APP_H */

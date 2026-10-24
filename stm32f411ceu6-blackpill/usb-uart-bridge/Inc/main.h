/**
 * @file    main.h
 * @brief   Included by the USB glue files (as in a CubeMX project).
 */
#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
void Error_Handler(void);

#endif /* MAIN_H */

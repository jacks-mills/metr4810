#include "main.h"
#include "start.h"

static volatile uint32_t startPressed = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == START_BUTTON_Pin)
    startPressed = 1;
}

uint32_t wait_for_start(uint32_t timeout_ms)
{
  if (HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin) == GPIO_PIN_SET)
    return timeout_ms;

  startPressed = 0;
  uint32_t start = HAL_GetTick();
  uint32_t elapsed = 0;

  while (!startPressed)
  {
    elapsed = HAL_GetTick() - start;
    if (elapsed >= timeout_ms)
      return 0;
    __WFI();
  }
  return timeout_ms - elapsed;
}

/**
 * @file    status_led_blackpill.c
 * @brief   Status LED on the Blackpill: PC13, lit when the pin is low.
 *
 *   idle     short blink every 2 s     manual   1 Hz blink
 *   auto     on                        stopped  2 Hz blink
 *   fault    fast flicker              fault_halt()  very fast, steady flash
 */
#include "main.h"
#include "status_led.h"

void status_led_show(enum Mode mode, bool startReady, uint32_t now)
{
    (void)startReady;                   /* no LED for it on this board */
    bool on;

    switch (mode) {
    case MODE_AUTO:    on = true;                  break;
    case MODE_MANUAL:  on = (now % 1000) < 500;    break;
    case MODE_STOPPED: on = (now % 500) < 250;     break;
    case MODE_FAULT:   on = (now % 200) < 100;     break;
    default:           on = (now % 2000) < 100;    break;
    }
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void status_led_fault_toggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

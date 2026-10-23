/**
 * @file    status_led_pcb.c
 * @brief   Status LEDs on the PCB: red PD0, orange PD1, green PD2, start PD3.
 *
 *   green    on = sequence running (auto); a short blink every 2 s = idle
 *   orange   on = paused, manual control
 *   red      on = stopped; flashing = fault
 *   start    on = the start button will start or continue the sequence
 *   all four flashing together, very fast = fault_halt() (stack overflow,
 *   heap full or hard fault; the debugger shows faultReason)
 *
 * Assumes each LED lights when its pin is high (pin -> resistor -> LED ->
 * GND). If yours light when the pin is low, set LED_ON to GPIO_PIN_RESET.
 */
#include "main.h"
#include "status_led.h"

#define LED_ON      GPIO_PIN_SET
#define LED_OFF     (LED_ON == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET)

static void set(GPIO_TypeDef *port, uint16_t pin, bool on)
{
    HAL_GPIO_WritePin(port, pin, on ? LED_ON : LED_OFF);
}

void status_led_show(enum Mode mode, bool startReady, uint32_t now)
{
    bool red = false, orange = false, green = false;

    switch (mode) {
    case MODE_AUTO:    green = true;                   break;
    case MODE_MANUAL:  orange = true;                  break;
    case MODE_STOPPED: red = true;                     break;
    case MODE_FAULT:   red = (now % 200) < 100;        break;
    default:           green = (now % 2000) < 100;     break;
    }

    set(LED_RED_GPIO_Port, LED_RED_Pin, red);
    set(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, orange);
    set(LED_GREEN_GPIO_Port, LED_GREEN_Pin, green);
    set(LED_START_GPIO_Port, LED_START_Pin, startReady);
}

void status_led_fault_toggle(void)
{
    static bool on;

    on = !on;                           /* all four together */
    set(LED_RED_GPIO_Port, LED_RED_Pin, on);
    set(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin, on);
    set(LED_GREEN_GPIO_Port, LED_GREEN_Pin, on);
    set(LED_START_GPIO_Port, LED_START_Pin, on);
}

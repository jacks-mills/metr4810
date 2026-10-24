/**
 * @file    main.c
 * @brief   Cutting head ramp test for the PCB (STM32F401VCT6). No serial
 *          link needed: it runs a fixed sequence from reset.
 *
 * Sequence:
 *   1. START_DELAY_MS (5 s): motor stopped, LED flashing. Stand clear.
 *   2. RAMP_UP_MS (10 s): stopped -> full speed.
 *   3. HOLD_TOP_MS (2 s): full speed.
 *   4. RAMP_DOWN_MS (10 s): full speed -> stopped.
 *   5. HOLD_STOPPED_MS (2 s): stopped, then back to 2, for ever.
 * The LED fades with the motor: off when stopped, brightest at full speed.
 *
 * Motor: the cutting head on the LEFT motor header.
 *   LEFT_PWM PA6 = TIM3 CH1, 20 kHz. Assumed to be the FIT0441 (the drill
 *   motor), whose speed input is inverted: pin high = stopped, low = full
 *   speed. The channel polarity is Low, so compare 0 = stopped, as in
 *   robot's drill.c. For a motor that runs when its input is high (like the
 *   DRI0002's E), set MOTOR_PWM_INVERTED to 0.
 *   LEFT_DIR PE0 = MOTOR_DIR_LEVEL (high = forward on the FIT0441).
 * The other motor outputs are held at stopped: RIGHT_PWM PA7 low (DRI0002
 * E off) and DRILL_PWM PB0 high (FIT0441 stopped).
 *
 * LED: on J7, PA9 = TIM1 CH2, 1 kHz. It has no series resistor, so while
 * the pin is high the current is limited only by the pin's own drive
 * strength. The PWM never has it on for more than LED_MAX_PERMILLE of the
 * time (25 %), half of what blinky's 1 s on / 1 s off gave it. Duty goes
 * with the square of brightness so the fade looks even. With a resistor
 * (220-330 ohm) in series, LED_MAX_PERMILLE can go up to 1000.
 *
 * Clock: the reset default, HSI 16 MHz with no PLL. APB1 = APB2 = 16 MHz,
 * so TIM1 and TIM3 both count at 16 MHz.
 *
 * Register level, no HAL, like uart-test.
 */
#include <stdint.h>

#include "stm32f4xx.h"

/* ---------------------------------------------------------------- settings */

#define START_DELAY_MS      5000u   /* after reset: motor stopped, LED flashing */
#define RAMP_UP_MS          10000u  /* stopped -> top speed */
#define HOLD_TOP_MS         2000u   /* at top speed */
#define RAMP_DOWN_MS        10000u  /* top speed -> stopped */
#define HOLD_STOPPED_MS     2000u   /* stopped, before the next ramp up */

#define MOTOR_TOP_PERMILLE  1000u   /* top of the ramp: 1000 = full speed */
#define MOTOR_PWM_INVERTED  1       /* 1: pin high = stopped (FIT0441), 0: pin high = run */
#define MOTOR_DIR_LEVEL     1       /* LEFT_DIR: 1 = high = forward on the FIT0441 */

#define LED_MAX_PERMILLE    250u    /* LED on for at most 25 % of each PWM period */
#define LED_ACTIVE_HIGH     1       /* 1: LED from PA9 to GND, 0: LED from 3.3 V to PA9 */

#define TIMER_CLOCK_HZ      16000000u
#define MOTOR_PERIOD        (TIMER_CLOCK_HZ / 20000u)   /* 20 kHz: 800 counts */
#define LED_PERIOD          (TIMER_CLOCK_HZ / 1000u)    /* 1 kHz: 16000 counts */
#define CYCLE_MS            (RAMP_UP_MS + HOLD_TOP_MS + RAMP_DOWN_MS + HOLD_STOPPED_MS)

_Static_assert(MOTOR_TOP_PERMILLE >= 1 && MOTOR_TOP_PERMILLE <= 1000, "MOTOR_TOP_PERMILLE: 1 to 1000");
_Static_assert(LED_MAX_PERMILLE <= 1000, "LED_MAX_PERMILLE: 0 to 1000");
_Static_assert(CYCLE_MS > 0, "the ramp and hold times can't all be 0");

/* ---------------------------------------------------------------- time */

static volatile uint32_t msTicks;

void SysTick_Handler(void)
{
    msTicks++;
}

/* ---------------------------------------------------------------- outputs */

/* Kept for the debugger: what the outputs were last set to (0 to 1000). */
static volatile uint32_t motorSpeed, ledBrightness;

/* speed: 0 = stopped to 1000 = full speed. */
static void motor_set(uint32_t speed)
{
    if (speed > 1000)
        speed = 1000;
    motorSpeed = speed;
    TIM3->CCR1 = MOTOR_PERIOD * speed / 1000;
}

/* brightness: 0 = off to 1000 = brightest allowed (LED_MAX_PERMILLE duty). */
static void led_set(uint32_t brightness)
{
    if (brightness > 1000)
        brightness = 1000;
    ledBrightness = brightness;
    TIM1->CCR2 = (uint32_t)((uint64_t)LED_PERIOD * LED_MAX_PERMILLE
                            * brightness * brightness / 1000000000u);
}

/* ---------------------------------------------------------------- sequence */

/* Motor speed (0 to 1000) at t ms into the repeating part (steps 2 to 5). */
static uint32_t cycle_speed(uint32_t t)
{
    if (t < RAMP_UP_MS)
        return MOTOR_TOP_PERMILLE * t / RAMP_UP_MS;
    t -= RAMP_UP_MS;
    if (t < HOLD_TOP_MS)
        return MOTOR_TOP_PERMILLE;
    t -= HOLD_TOP_MS;
    if (t < RAMP_DOWN_MS)
        return MOTOR_TOP_PERMILLE * (RAMP_DOWN_MS - t) / RAMP_DOWN_MS;
    return 0;
}

/* ---------------------------------------------------------------- set-up */

static void hw_init(void)
{
    SystemCoreClockUpdate();            /* 16 MHz (HSI) */

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOEEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;                 /* let the clock enables take effect */

    /* Other motors held stopped: RIGHT_PWM PA7 low, DRILL_PWM PB0 high.
     * Level first, then output mode, so there's no glitch. */
    GPIOA->BSRR = GPIO_BSRR_BR7;
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER7) | (1u << GPIO_MODER_MODER7_Pos);
    GPIOB->BSRR = GPIO_BSRR_BS0;
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER0) | (1u << GPIO_MODER_MODER0_Pos);

    /* LEFT_DIR PE0. */
    GPIOE->BSRR = MOTOR_DIR_LEVEL ? GPIO_BSRR_BS0 : GPIO_BSRR_BR0;
    GPIOE->MODER = (GPIOE->MODER & ~GPIO_MODER_MODER0) | (1u << GPIO_MODER_MODER0_Pos);

    /* TIM3 CH1 -> LEFT_PWM PA6 (AF2): PWM mode 1, compare 0 = stopped. The
     * timer is running before the pin is handed to it, so the pin goes
     * straight to the stopped level. */
    TIM3->PSC = 0;
    TIM3->ARR = MOTOR_PERIOD - 1;
    TIM3->CCR1 = 0;
    TIM3->CCMR1 = (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM3->CCER = TIM_CCER_CC1E | (MOTOR_PWM_INVERTED ? TIM_CCER_CC1P : 0);
    TIM3->EGR = TIM_EGR_UG;
    TIM3->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFSEL6) | (2u << GPIO_AFRL_AFSEL6_Pos);
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER6) | (2u << GPIO_MODER_MODER6_Pos);

    /* TIM1 CH2 -> LED PA9 (AF1): PWM mode 1, compare 0 = off. TIM1 is an
     * advanced timer, so its outputs also need MOE. */
    TIM1->PSC = 0;
    TIM1->ARR = LED_PERIOD - 1;
    TIM1->CCR2 = 0;
    TIM1->CCMR1 = (6u << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;
    TIM1->CCER = TIM_CCER_CC2E | (LED_ACTIVE_HIGH ? 0 : TIM_CCER_CC2P);
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->EGR = TIM_EGR_UG;
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~GPIO_AFRH_AFSEL9) | (1u << GPIO_AFRH_AFSEL9_Pos);
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER9) | (2u << GPIO_MODER_MODER9_Pos);

    SysTick_Config(SystemCoreClock / 1000);     /* 1 ms tick */
}

/* ---------------------------------------------------------------- main */

int main(void)
{
    hw_init();

    uint32_t start = msTicks;
    for (;;) {
        uint32_t now = msTicks;
        uint32_t t = now - start;

        if (t < START_DELAY_MS) {
            motor_set(0);
            led_set(t % 500 < 100 ? 1000 : 0);      /* 0.1 s flash every 0.5 s */
        } else {
            uint32_t speed = cycle_speed((t - START_DELAY_MS) % CYCLE_MS);
            motor_set(speed);
            led_set(speed * 1000 / MOTOR_TOP_PERMILLE);     /* brightest at the top */
        }

        /* Wait for the next tick. Busy-waiting, not __WFI(): sleeping can
         * stop the ST-Link connecting the next time you flash. */
        while (msTicks == now) {
        }
    }
}

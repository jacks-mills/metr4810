/* Motor test for the PCB (STM32F401VCT6): serial commands for the motor on
 * J1, a PWM test, the ST3215 servo on J7 pin 3 and status LEDs on J5.
 * Console: J7 pins 5/6, 115200 8N1 (software UART on TIM1). Type help.
 *
 * AI-generated code: written with Claude Opus 5.5 (claude-opus-5-5), an AI
 * assistant by Anthropic, October 2026. */
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32f4xx.h"
#include "autorun.h"
#include "servo.h"
#include "swuart.h"

/* ---- Settings ---- */

#define BAUD                115200u
#define PULSE_MAX_MS        60000u
#define REVERSE_PAUSE_MS    100u    /* E off this long when reversing */
#define DIR_AT_RESET        0       /* M at reset */
#define E_ON_LEVEL          1       /* E level that runs the motor */

#define AUTO_PULSE_MS       500u    /* automatic mode defaults */
#define AUTO_WAIT_MS        500u
#define AUTO_TIMEOUT_MS     5000u
#define AUTO_COUNT          0u
#define AUTO_REST_MS        1000u
#define AUTO_CYCLES         0u
#define AUTO_FIRST_DIR      0
#define AUTO_MAX_MS         3600000u    /* 1 hour */
#define AUTO_MAX_COUNT      1000000u    /* most pulses each way, or cycles */

#define LIMIT_ACTIVE_HIGH   1       /* pressed = high */
#define LIMIT_DEBOUNCE_MS   20u

#define LED_ACTIVE_HIGH     1       /* LED from the pin through a resistor to GND */
#define HEARTBEAT_MS        1000u   /* heartbeat period */
#define LAMP_TEST_MS        250u    /* each LED lit in turn at start-up */
#define SERIAL_FLASH_MS     30u     /* serial LED on per byte received */
#define SERIAL_ERROR_MS     2000u   /* ...and after a line error */

#define WATCHDOG_MS         1000u   /* about 1 s (LSI is 17-47 kHz) */

#define LINE_MAX            80u     /* longest command line */
#define HISTORY_SIZE        16u     /* commands kept for the Up arrow */
#define ESC_WAIT_MS         20u     /* lone Esc after this long = the Esc key */
#define RX_SIZE             1024u   /* receive ring, power of two */
#define TX_SIZE             4096u   /* transmit ring, power of two */

#define PWM_DEFAULT_MHZ     1000000u    /* 1 kHz, in thousandths of a Hz */
#define PWM_DEFAULT_DUTY    50000u      /* 50 %, in thousandths of a % */
#define PWM_MEASURE_MS      200u        /* how long 'pwm' samples the pins */

#define CONSOLE_IRQ_PRIORITY 0u     /* TIM1 (software console UART): highest */
#define IRQ_PRIORITY        1u      /* SysTick */

#define SERVO_BAUD          1000000u    /* the servo's factory setting */
#define SERVO_SPEED         500u        /* steps/s (4096 steps = 360 deg) */
#define SERVO_ACC           50u         /* x100 steps/s^2 */

_Static_assert((RX_SIZE & (RX_SIZE - 1)) == 0, "RX_SIZE must be a power of two");
_Static_assert((TX_SIZE & (TX_SIZE - 1)) == 0, "TX_SIZE must be a power of two");
_Static_assert(WATCHDOG_MS >= 100 && WATCHDOG_MS <= 4096, "WATCHDOG_MS: 100 to 4096");
_Static_assert(AUTO_REST_MS >= REVERSE_PAUSE_MS, "the rest before reversing must cover REVERSE_PAUSE_MS");

/* ---- Pins ---- */

struct Pin {
    GPIO_TypeDef *port;
    uint32_t n;                         /* 0 to 15 */
};

/* Motor: J1 */
static const struct Pin PIN_E = {GPIOA, 0};         /* J1 pin 5: DRI0002 E (J1 pin 3 is dead) */
static const struct Pin PIN_M = {GPIOE, 0};         /* J1 pin 4: DRI0002 M */

/* Limit switches */
static const struct Pin PIN_LIMIT_MAX = {GPIOC, 2}; /* EXT_MAX */
static const struct Pin PIN_LIMIT_MIN = {GPIOC, 3}; /* EXT_MIN */

/* Serial: J7 */
static const struct Pin PIN_TX = {GPIOA, 9};        /* J7 pin 5: software UART TX (TIM1_CH2) */
static const struct Pin PIN_RX = {GPIOA, 10};       /* J7 pin 6: software UART RX (TIM1_CH3) */

/* Servo: J7 pin 3 */
static const struct Pin PIN_SERVO = {GPIOB, 6};     /* USART1_TX (AF7), half-duplex */

/* LEDs: each on a J5 pin and a board LED pin (PD0-PD3, for J4) */
enum Led { LED_HEARTBEAT, LED_DRIVE, LED_DIR, LED_SERIAL, LED_COUNT };

static const struct {
    const char *what;
    struct Pin j5;
    const char *j5Name;
    struct Pin board;
    const char *boardName;
} leds[LED_COUNT] = {
    [LED_HEARTBEAT] = {"heartbeat", {GPIOB, 12}, "J5 pin 3 (PB12)", {GPIOD, 2}, "green PD2"},
    [LED_DRIVE]     = {"drive",     {GPIOB, 13}, "J5 pin 4 (PB13)", {GPIOD, 1}, "orange PD1"},
    [LED_DIR]       = {"direction", {GPIOB, 14}, "J5 pin 5 (PB14)", {GPIOD, 3}, "start PD3"},
    [LED_SERIAL]    = {"serial",    {GPIOB, 15}, "J5 pin 6 (PB15)", {GPIOD, 0}, "red PD0"},
};

static inline void pin_write(struct Pin p, bool high)
{
    p.port->BSRR = high ? (1u << p.n) : (1u << (p.n + 16u));   /* atomic */
}

static inline bool pin_read(struct Pin p)           /* the level on the pin */
{
    return (p.port->IDR >> p.n) & 1u;
}

static inline bool pin_driven(struct Pin p)         /* the level we're driving it to */
{
    return (p.port->ODR >> p.n) & 1u;
}

/* Push-pull output, set to its level first so there's no glitch. */
static void pin_output(struct Pin p, bool high)
{
    pin_write(p, high);
    p.port->OTYPER &= ~(1u << p.n);
    p.port->OSPEEDR &= ~(3u << (2u * p.n));
    p.port->PUPDR &= ~(3u << (2u * p.n));
    p.port->MODER = (p.port->MODER & ~(3u << (2u * p.n))) | (1u << (2u * p.n));
}

static void pin_input(struct Pin p, uint32_t pull)  /* pull: 0 none, 1 up, 2 down */
{
    p.port->PUPDR = (p.port->PUPDR & ~(3u << (2u * p.n))) | (pull << (2u * p.n));
    p.port->MODER &= ~(3u << (2u * p.n));
}

static void pin_alternate(struct Pin p, uint32_t af, uint32_t pull)
{
    volatile uint32_t *afr = &p.port->AFR[p.n / 8u];
    uint32_t shift = 4u * (p.n % 8u);
    *afr = (*afr & ~(0xFu << shift)) | (af << shift);
    p.port->OTYPER &= ~(1u << p.n);
    p.port->OSPEEDR = (p.port->OSPEEDR & ~(3u << (2u * p.n))) | (1u << (2u * p.n));   /* medium */
    p.port->PUPDR = (p.port->PUPDR & ~(3u << (2u * p.n))) | (pull << (2u * p.n));
    p.port->MODER = (p.port->MODER & ~(3u << (2u * p.n))) | (2u << (2u * p.n));
}

static uint32_t irq_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void irq_unlock(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/* ---- Time ---- */

static volatile uint32_t msTicks;

static void delay_ms(uint32_t ms)       /* start-up only; feeds the watchdog */
{
    uint32_t start = msTicks;
    while (msTicks - start < ms)
        IWDG->KR = 0xAAAA;
}

/* ---- PWM test outputs ----
 * Header pins a timer can drive. Pins on one timer share its frequency.
 * A pin that's off is an input, except the motor's E, which goes back to
 * the drive. */

enum { TIM_2, TIM_3, TIM_4, TIMER_COUNT };

static const struct {
    const char *name;
    TIM_TypeDef *tim;
    bool is32;                          /* 32-bit (TIM2), else 16-bit */
} pwmTimers[TIMER_COUNT] = {
    [TIM_2] = {"TIM2", TIM2, true},
    [TIM_3] = {"TIM3", TIM3, false},
    [TIM_4] = {"TIM4", TIM4, false},
};

#define PWM_E   1                       /* the motor's E: J1 pin 5, PA0 */

static const struct {
    const char *name;                   /* header pin */
    const char *chip;
    const char *net;
    struct Pin pin;
    uint32_t af;
    uint8_t timer;
    uint8_t ch;                         /* channel 1 to 4 */
    bool inAll;                         /* included in 'pwm on' and 'all' */
} pwmChannels[] = {
              {"j1.3", "PA6", "LEFT_PWM",  {GPIOA, 6}, 2, TIM_3, 1, true},
    [PWM_E] = {"j1.5", "PA0", "LEFT_FG",   {GPIOA, 0}, 1, TIM_2, 1, false},
              {"j1.6", "PB0", "DRILL_PWM", {GPIOB, 0}, 2, TIM_3, 3, true},
              {"j2.3", "PA7", "RIGHT_PWM", {GPIOA, 7}, 2, TIM_3, 2, true},
              {"j2.5", "PA1", "RIGHT_FG",  {GPIOA, 1}, 1, TIM_2, 2, true},
              {"j2.8", "PA2", "SERVO_SIG", {GPIOA, 2}, 1, TIM_2, 3, true},
              {"j7.4", "PB7", "I2C1_SDA",  {GPIOB, 7}, 2, TIM_4, 2, true},
};
#define PWM_COUNT ((unsigned)(sizeof pwmChannels / sizeof pwmChannels[0]))

static volatile bool pwmOn[PWM_COUNT];
static uint32_t pwmDuty[PWM_COUNT];     /* thousandths of a % */
static bool pwmInverted[PWM_COUNT];
static uint32_t timerClockHz;           /* what TIM2-TIM4 count at */

static void pwm_pin_on(unsigned i)
{
    uint32_t key = irq_lock();
    pin_alternate(pwmChannels[i].pin, pwmChannels[i].af, 0);
    pwmOn[i] = true;
    irq_unlock(key);
}

/* From main or an interrupt. */
static void pwm_pin_off(unsigned i)
{
    uint32_t key = irq_lock();
    if (i == PWM_E)
        pin_output(pwmChannels[i].pin, !E_ON_LEVEL);    /* back to the drive (off) */
    else
        pin_input(pwmChannels[i].pin, 0);
    pwmOn[i] = false;
    irq_unlock(key);
}

/* Turns every PWM pin off. Returns true if any was on. */
static bool pwm_all_off(void)
{
    bool any = false;
    for (unsigned i = 0; i < PWM_COUNT; i++) {
        if (pwmOn[i]) {
            pwm_pin_off(i);
            any = true;
        }
    }
    return any;
}

/* The drive takes E back from the PWM test. */
static void pwm_release_e(void)
{
    if (pwmOn[PWM_E])
        pwm_pin_off(PWM_E);
}

/* ---- Drive ----
 * Drive state changes only in SysTick or with interrupts off; drive_apply()
 * sets E from it. */

enum DriveMode { MODE_OFF, MODE_ON, MODE_PULSE, MODE_AUTO };

/* Events from the interrupts to the main loop */
#define EV_PULSE_DONE   (1u << 0)
#define EV_STOP_KEY     (1u << 1)       /* Esc or Ctrl-C stopped the drive */
#define EV_STOP_SERIAL  (1u << 2)       /* a serial line error stopped the drive */
#define EV_AUTO_REVERSE (1u << 3)       /* automatic mode has just reversed */
#define EV_AUTO_DONE    (1u << 4)       /* automatic mode has done all its cycles */

static volatile enum DriveMode mode = MODE_OFF;
static volatile uint32_t pulseMs;       /* length of the current pulse */
static volatile uint32_t pulseLeftMs;   /* E-on time still to go */
static volatile bool pulseWaitTick;     /* pulse starts on the next tick */
static volatile uint32_t pauseLeftMs;   /* E held off after a direction change */
static volatile uint32_t stoppedForMs = REVERSE_PAUSE_MS;  /* how long E has been off (capped) */
static volatile bool dirHigh = DIR_AT_RESET;
static volatile uint32_t events;
static volatile uint32_t stopRequest;   /* stops from the console interrupt, done by SysTick */
static volatile bool escPending;        /* Esc received: might be the start of an arrow key */
static volatile uint32_t escAtMs;

/* Automatic mode: run by SysTick; main uses it only with interrupts off. */
static const struct AutoSettings autoDefaults = {
    AUTO_PULSE_MS, AUTO_WAIT_MS, AUTO_TIMEOUT_MS, AUTO_COUNT, AUTO_REST_MS, AUTO_CYCLES, AUTO_FIRST_DIR,
};
static struct AutoSettings autoSet = {
    AUTO_PULSE_MS, AUTO_WAIT_MS, AUTO_TIMEOUT_MS, AUTO_COUNT, AUTO_REST_MS, AUTO_CYCLES, AUTO_FIRST_DIR,
};
static struct AutoRun autoRun;

static bool motor_should_run(void)
{
    if (mode == MODE_OFF || pauseLeftMs > 0 || pulseWaitTick)
        return false;
    return mode != MODE_AUTO || autorun_e_on(&autoRun);
}

static void drive_apply(void)
{
    bool run = motor_should_run();
    pin_write(PIN_E, run ? E_ON_LEVEL : !E_ON_LEVEL);
}

static bool motor_running(void)         /* from the pin we're driving */
{
    return pin_driven(PIN_E) == (E_ON_LEVEL != 0);
}

/* Drive and all PWM off. From an interrupt. */
static void stop_all_isr(uint32_t event)
{
    bool wasOn = mode != MODE_OFF;
    mode = MODE_OFF;
    pulseWaitTick = false;
    pulseLeftMs = 0;
    drive_apply();
    if (pwm_all_off())
        wasOn = true;
    if (wasOn)
        events |= event;
}

static void drive_set(bool on)
{
    pwm_release_e();
    uint32_t key = irq_lock();
    mode = on ? MODE_ON : MODE_OFF;
    pulseWaitTick = false;
    pulseLeftMs = 0;
    drive_apply();
    irq_unlock(key);
}

/* E on for ms, then off (timed by SysTick). */
static void drive_pulse(uint32_t ms)
{
    pwm_release_e();
    uint32_t key = irq_lock();
    bool wasRunning = motor_should_run();
    mode = MODE_PULSE;
    pulseMs = ms;
    pulseLeftMs = ms;
    pulseWaitTick = !wasRunning;
    drive_apply();
    irq_unlock(key);
}

/* Sets M; holds E off for REVERSE_PAUSE_MS if the motor may be turning. */
static bool dir_change_locked(bool high)
{
    bool paused = false;
    if (high != dirHigh) {
        if (motor_should_run() || pauseLeftMs > 0 || stoppedForMs < REVERSE_PAUSE_MS) {
            pauseLeftMs = REVERSE_PAUSE_MS;
            drive_apply();              /* E off before M changes */
            paused = (mode != MODE_OFF);
        }
        dirHigh = high;
        pin_write(PIN_M, high);
    }
    return paused;
}

static bool drive_set_dir(bool high)
{
    uint32_t key = irq_lock();
    bool paused = dir_change_locked(high);
    irq_unlock(key);
    return paused;
}

/* Starts automatic mode from its first pulse. True if paused to reverse. */
static bool auto_start(void)
{
    pwm_release_e();
    uint32_t key = irq_lock();
    mode = MODE_AUTO;
    pulseLeftMs = 0;
    autorun_begin(&autoRun, &autoSet);
    pulseWaitTick = true;
    bool paused = dir_change_locked(autoSet.firstDir);
    drive_apply();
    irq_unlock(key);
    return paused;
}

/* SysTick, once per ms, when not paused. */
static void auto_tick(void)
{
    if (pulseWaitTick) {
        pulseWaitTick = false;          /* the first pulse begins now */
        return;
    }
    enum AutoEvent ev = autorun_tick(&autoRun, &autoSet);
    if (ev == AUTO_REVERSE) {
        /* E has been off for the whole rest, so M can change now. */
        if (stoppedForMs < REVERSE_PAUSE_MS)
            pauseLeftMs = REVERSE_PAUSE_MS;
        dirHigh = !dirHigh;
        pin_write(PIN_M, dirHigh);
        events |= EV_AUTO_REVERSE;
    } else if (ev == AUTO_DONE) {
        mode = MODE_OFF;
        events |= EV_AUTO_DONE;
    }
}

void SysTick_Handler(void)
{
    msTicks++;

    /* Stops asked for by the console interrupt, and a lone Esc. */
    uint32_t key = irq_lock();
    uint32_t req = stopRequest;
    stopRequest = 0;
    if (escPending && msTicks - escAtMs >= ESC_WAIT_MS) {
        escPending = false;
        req |= EV_STOP_KEY;
    }
    irq_unlock(key);
    if (req & EV_STOP_KEY)
        stop_all_isr(EV_STOP_KEY);
    if (req & EV_STOP_SERIAL)
        stop_all_isr(EV_STOP_SERIAL);

    if (pauseLeftMs > 0) {
        pauseLeftMs--;                  /* a pulse doesn't count down while paused */
    } else if (mode == MODE_PULSE) {
        if (pulseWaitTick) {
            pulseWaitTick = false;      /* E goes on now */
        } else if (pulseLeftMs <= 1) {
            pulseLeftMs = 0;
            mode = MODE_OFF;
            events |= EV_PULSE_DONE;
        } else {
            pulseLeftMs--;
        }
    } else if (mode == MODE_AUTO) {
        auto_tick();
    }
    drive_apply();

    if (motor_should_run())
        stoppedForMs = 0;
    else if (stoppedForMs < REVERSE_PAUSE_MS)
        stoppedForMs++;
}

/* ---- Console: software UART ----
 * 115200 8N1 on J7 pins 5/6, timed by TIM1:
 *   TX, PA9 = TIM1_CH2 output compare: the timer sets each bit edge.
 *   RX, PA10 = TIM1_CH3 captures the start bit; CH4 samples each bit.
 * Bit-level logic in swuart.h. */

static volatile uint8_t rxBuf[RX_SIZE];
static volatile uint32_t rxHead, rxTail;    /* head: interrupt, tail: main */
static volatile uint8_t txBuf[TX_SIZE];
static volatile uint32_t txHead, txTail;    /* head: main, tail: interrupt */

static volatile struct {
    uint32_t framing, noise, lost;
} serialErrors;

static volatile uint32_t lastRxMs, lastErrorMs;
static volatile bool rxSeen, errorSeen;

static uint16_t bitTicks;                   /* TIM1 counts per bit */
static uint16_t sampleEarly;                /* sample this much before mid-bit */
static struct SwuRx swRx;
static struct SwuTx swTx;
static volatile bool txBusy;

#define OC_ACTIVE_ON_MATCH      1u          /* TIM1 output compare modes for CH2 */
#define OC_INACTIVE_ON_MATCH    2u
#define OC_FORCE_ACTIVE         5u

static void tx_oc_mode(uint32_t ocMode)
{
    TIM1->CCMR1 = (TIM1->CCMR1 & ~TIM_CCMR1_OC2M) | (ocMode << TIM_CCMR1_OC2M_Pos);
}

/* A received byte or error, from the TIM1 interrupt. Stops go to SysTick. */
static void console_rx_result(enum SwuRxResult r, uint8_t byte)
{
    bool problem = false;

    if (r == SWU_BYTE) {
        lastRxMs = msTicks;
        rxSeen = true;
        /* Ctrl-C stops. Esc stops unless [ or O follows (an arrow key). */
        if (escPending) {
            escPending = false;
            if (byte != '[' && byte != 'O')
                stopRequest |= EV_STOP_KEY;
        }
        if (byte == 0x1B) {
            escAtMs = msTicks;
            escPending = true;
        } else if (byte == 0x03) {
            stopRequest |= EV_STOP_KEY;
        }
        if (rxHead - rxTail < RX_SIZE) {
            rxBuf[rxHead % RX_SIZE] = byte;
            rxHead++;
        } else {
            serialErrors.lost++;
            problem = true;
        }
    } else if (r == SWU_GLITCH) {
        serialErrors.noise++;
        problem = true;
    } else {                                /* framing error or break */
        serialErrors.framing++;
        problem = true;
    }

    if (problem) {
        lastErrorMs = msTicks;
        errorSeen = true;
        stopRequest |= EV_STOP_SERIAL;
    }
}

/* Starts sending the next byte. Interrupts off, or the TIM1 interrupt. */
static void tx_start_locked(void)
{
    swTx.byte = txBuf[txTail % TX_SIZE];
    txTail++;
    swTx.bit = 0;
    tx_oc_mode(OC_INACTIVE_ON_MATCH);
    TIM1->CCR2 = (uint16_t)(TIM1->CNT + 40u);
    TIM1->SR = ~TIM_SR_CC2IF;
    TIM1->DIER |= TIM_DIER_CC2IE;
    txBusy = true;
}

/* Start bit at edge: sample each bit near its middle. */
static void rx_start_frame(uint16_t edge)
{
    swRx.bit = 0;
    TIM1->CCR4 = (uint16_t)(edge + bitTicks / 2u - sampleEarly);
    TIM1->SR = ~TIM_SR_CC4IF;
    TIM1->DIER = (TIM1->DIER & ~TIM_DIER_CC3IE) | TIM_DIER_CC4IE;
}

void TIM1_CC_IRQHandler(void)
{
    uint32_t sr = TIM1->SR;
    uint32_t dier = TIM1->DIER;

    /* receive: the falling edge of a start bit */
    if ((dier & TIM_DIER_CC3IE) && (sr & TIM_SR_CC3IF)) {
        uint16_t edge = (uint16_t)TIM1->CCR3;       /* reading it clears CC3IF */
        TIM1->SR = ~TIM_SR_CC3OF;
        rx_start_frame(edge);
    }

    /* receive: the middle of a bit */
    if ((dier & TIM_DIER_CC4IE) && (sr & TIM_SR_CC4IF)) {
        TIM1->SR = ~TIM_SR_CC4IF;
        uint8_t byte = 0;
        enum SwuRxResult r = swu_rx_sample(&swRx, pin_read(PIN_RX), &byte);
        if (r == SWU_MORE) {
            TIM1->CCR4 = (uint16_t)(TIM1->CCR4 + bitTicks);
        } else {
            uint16_t sampledAt = (uint16_t)TIM1->CCR4;
            console_rx_result(r, byte);
            /* The next start bit may already have been captured. */
            bool next = false;
            uint16_t edge = 0;
            if (TIM1->SR & TIM_SR_CC3IF) {
                edge = (uint16_t)TIM1->CCR3;
                next = (int16_t)(edge - sampledAt) > 0;
            }
            TIM1->SR = ~TIM_SR_CC3OF;
            if (next)
                rx_start_frame(edge);
            else
                TIM1->DIER = (TIM1->DIER & ~TIM_DIER_CC4IE) | TIM_DIER_CC3IE;
        }
    }

    /* send: a bit edge has just happened on PA9 */
    if ((dier & TIM_DIER_CC2IE) && (sr & TIM_SR_CC2IF)) {
        TIM1->SR = ~TIM_SR_CC2IF;
        bool have = txTail != txHead, took;
        enum SwuTxNext x = swu_tx_after_edge(&swTx, have, have ? txBuf[txTail % TX_SIZE] : 0, &took);
        if (took)
            txTail++;
        if (x == SWU_TX_IDLE) {
            tx_oc_mode(OC_FORCE_ACTIVE);    /* idle: high */
            TIM1->DIER &= ~TIM_DIER_CC2IE;
            txBusy = false;
            if (txTail != txHead)
                tx_start_locked();
        } else {
            tx_oc_mode(x == SWU_TX_HIGH ? OC_ACTIVE_ON_MATCH : OC_INACTIVE_ON_MATCH);
            TIM1->CCR2 = (uint16_t)(TIM1->CCR2 + bitTicks);
        }
    }
}

static bool uart_read(uint8_t *byte)
{
    if (rxTail == rxHead)
        return false;
    *byte = rxBuf[rxTail % RX_SIZE];
    rxTail++;
    return true;
}

static void uart_putc(char c)
{
    /* Ring full: wait for the interrupt to send some. */
    while (txHead - txTail >= TX_SIZE) {
    }
    txBuf[txHead % TX_SIZE] = (uint8_t)c;
    uint32_t key = irq_lock();
    txHead++;
    if (!txBusy)
        tx_start_locked();
    irq_unlock(key);
}

static void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

static void uart_write(const char *s, size_t len)
{
    while (len--)
        uart_putc(*s++);
}

static void uart_put_u64(uint64_t v)
{
    char digits[20];
    int n = 0;
    do {
        digits[n++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v);
    while (n)
        uart_putc(digits[--n]);
}

static void uart_put_u32(uint32_t v)
{
    char digits[10];
    int n = 0;
    do {
        digits[n++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v);
    while (n)
        uart_putc(digits[--n]);
}

/* ---- Set-up ---- */

static const char *resetCause;

static void read_reset_cause(void)
{
    uint32_t csr = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;           /* clear the flags for next time */

    if (csr & RCC_CSR_IWDGRSTF)
        resetCause = "watchdog (the program stopped running)";
    else if (csr & RCC_CSR_WWDGRSTF)
        resetCause = "window watchdog";
    else if (csr & RCC_CSR_LPWRRSTF)
        resetCause = "low-power";
    else if (csr & RCC_CSR_SFTRSTF)
        resetCause = "software (after flashing, or after a fault)";
    else if (csr & RCC_CSR_PORRSTF)
        resetCause = "power-on";
    else if (csr & RCC_CSR_BORRSTF)
        resetCause = "brown-out (the 3.3 V supply dipped)";
    else if (csr & RCC_CSR_PINRSTF)
        resetCause = "reset pin (NRST)";
    else
        resetCause = "unknown";
}

/* 84 MHz from HSI via the PLL. Stays on 16 MHz if the PLL doesn't lock. */
static void clock_init(void)
{
    FLASH->ACR = FLASH_ACR_LATENCY_2WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
    for (uint32_t n = 0; n < 1000u && (FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2WS; n++) {
    }
    if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2WS)
        return;

    RCC->PLLCFGR = RCC_PLLCFGR_PLLSRC_HSI | (16u << RCC_PLLCFGR_PLLM_Pos) |
                   (336u << RCC_PLLCFGR_PLLN_Pos) | (1u << RCC_PLLCFGR_PLLP_Pos) |    /* P: 1 = /4 */
                   (7u << RCC_PLLCFGR_PLLQ_Pos);
    RCC->CR |= RCC_CR_PLLON;
    for (uint32_t n = 0; n < 100000u && !(RCC->CR & RCC_CR_PLLRDY); n++) {
    }
    if (!(RCC->CR & RCC_CR_PLLRDY))
        return;                         /* stay on HSI */

    RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) | RCC_CFGR_PPRE1_DIV2;
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    for (uint32_t n = 0; n < 100000u && (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL; n++) {
    }
}

static void motor_pins_init(void)
{
    /* First thing: E off, M at its reset level. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOEEN;
    (void)RCC->AHB1ENR;                 /* let the clock enable take effect */
    pin_output(PIN_E, !E_ON_LEVEL);
    pin_output(PIN_M, DIR_AT_RESET);
}

static void other_pins_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN;
    (void)RCC->AHB1ENR;

    for (int i = 0; i < LED_COUNT; i++) {
        pin_output(leds[i].j5, !LED_ACTIVE_HIGH);
        pin_output(leds[i].board, !LED_ACTIVE_HIGH);
    }
    pin_input(PIN_LIMIT_MAX, LIMIT_ACTIVE_HIGH ? 2u : 1u);
    pin_input(PIN_LIMIT_MIN, LIMIT_ACTIVE_HIGH ? 2u : 1u);
}

static void console_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC->APB2ENR;

    /* TIM1 clock: PCLK2, doubled if APB2 is divided. */
    static const uint8_t apbShift[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    uint32_t pclk2 = SystemCoreClock >> apbShift[ppre2];
    uint32_t timerClock = ppre2 >= 4u ? pclk2 * 2u : pclk2;
    bitTicks = (uint16_t)((timerClock + BAUD / 2u) / BAUD);        /* 729 at 84 MHz */
    sampleEarly = (uint16_t)(bitTicks / 8u);

    TIM1->CR1 = 0;
    TIM1->PSC = 0;
    TIM1->ARR = 0xFFFF;                 /* free-running */
    /* CH2 (TX): output compare, idle high. */
    TIM1->CCMR1 = OC_FORCE_ACTIVE << TIM_CCMR1_OC2M_Pos;
    /* CH3 (RX): filtered input capture. CH4: compare for the sample times. */
    uint32_t filter = timerClock >= 48000000u ? 8u : 3u;
    TIM1->CCMR2 = (1u << TIM_CCMR2_CC3S_Pos) | (filter << TIM_CCMR2_IC3F_Pos);
    TIM1->CCER = TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC3P;   /* CH3: falling edges */
    TIM1->BDTR = TIM_BDTR_MOE;          /* outputs need MOE on TIM1 */
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR = 0;
    TIM1->DIER = TIM_DIER_CC3IE;        /* wait for a start bit */
    TIM1->CR1 = TIM_CR1_CEN;

    /* Pins after the timer, so TX starts idle. RX pulled up. */
    pin_alternate(PIN_TX, 1, 0);
    pin_alternate(PIN_RX, 1, 1);

    NVIC_SetPriority(TIM1_CC_IRQn, CONSOLE_IRQ_PRIORITY);
    NVIC_EnableIRQ(TIM1_CC_IRQn);
}

static void watchdog_start(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;     /* paused while debugging */
    IWDG->KR = 0xCCCC;                  /* start (and the LSI) */
    IWDG->KR = 0x5555;                  /* unlock PR and RLR */
    IWDG->PR = 3;                       /* about 1 kHz */
    IWDG->RLR = WATCHDOG_MS - 1u;
    for (uint32_t n = 0; n < 1000000u && IWDG->SR != 0; n++) {
    }                                   /* wait for the new values to load */
    IWDG->KR = 0xAAAA;
}

/* ---- PWM timers ---- */

static volatile uint32_t *pwm_ccr(unsigned i)
{
    TIM_TypeDef *t = pwmTimers[pwmChannels[i].timer].tim;
    switch (pwmChannels[i].ch) {
    case 1:  return &t->CCR1;
    case 2:  return &t->CCR2;
    case 3:  return &t->CCR3;
    default: return &t->CCR4;
    }
}

static uint64_t pwm_period(unsigned ti)         /* counts per period: ARR + 1 */
{
    return (uint64_t)pwmTimers[ti].tim->ARR + 1u;
}

/* Compare value for pin i from its duty, rounded. */
static void pwm_apply_duty(unsigned i)
{
    uint64_t period = pwm_period(pwmChannels[i].timer);
    uint64_t ccr = ((uint64_t)pwmDuty[i] * period + 50000u) / 100000u;
    *pwm_ccr(i) = (uint32_t)(ccr > 0xFFFFFFFFu ? 0xFFFFFFFFu : ccr);   /* ccr = period: 100 % */
}

static void pwm_apply_polarity(unsigned i)
{
    TIM_TypeDef *t = pwmTimers[pwmChannels[i].timer].tim;
    uint32_t bit = TIM_CCER_CC1P << (4u * (pwmChannels[i].ch - 1u));
    t->CCER = pwmInverted[i] ? (t->CCER | bit) : (t->CCER & ~bit);
}

/* Raw prescaler and auto-reload; duties are kept. */
static void pwm_timer_set(unsigned ti, uint32_t psc, uint32_t arr)
{
    TIM_TypeDef *t = pwmTimers[ti].tim;
    t->PSC = psc;
    t->ARR = arr;                       /* takes effect with the UG below */
    t->EGR = TIM_EGR_UG;
    for (unsigned i = 0; i < PWM_COUNT; i++)
        if (pwmChannels[i].timer == ti)
            pwm_apply_duty(i);
}

/* Sets a timer as close to milliHz as it can. False if out of range. */
static bool pwm_timer_freq(unsigned ti, uint64_t milliHz)
{
    if (milliHz == 0 || milliHz > (uint64_t)timerClockHz * 500u)
        return false;                   /* above the timer clock / 2 */
    uint64_t total = ((uint64_t)timerClockHz * 1000u + milliHz / 2u) / milliHz;    /* counts per period */
    uint64_t maxPeriod = pwmTimers[ti].is32 ? 0x100000000ull : 0x10000ull;
    if (total < 2u)
        return false;                   /* faster than the timer clock / 2 */
    uint64_t presc = (total + maxPeriod - 1u) / maxPeriod;
    if (presc > 0x10000u)
        return false;                   /* slower than the prescaler can reach */
    uint64_t period = (total + presc / 2u) / presc;
    if (period > maxPeriod)
        period = maxPeriod;
    if (period < 2u)
        period = 2u;
    pwm_timer_set(ti, (uint32_t)(presc - 1u), (uint32_t)(period - 1u));
    return true;
}

static uint64_t pwm_timer_milliHz(unsigned ti)  /* what it's actually running at */
{
    uint64_t counts = ((uint64_t)pwmTimers[ti].tim->PSC + 1u) * pwm_period(ti);
    return ((uint64_t)timerClockHz * 1000u + counts / 2u) / counts;
}

/* PWM defaults: all off, 1 kHz, 50 %, normal polarity. */
static void pwm_defaults(void)
{
    pwm_all_off();
    for (unsigned i = 0; i < PWM_COUNT; i++) {
        pwmDuty[i] = PWM_DEFAULT_DUTY;
        pwmInverted[i] = false;
        pwm_apply_polarity(i);
    }
    for (unsigned ti = 0; ti < TIMER_COUNT; ti++)
        pwm_timer_freq(ti, PWM_DEFAULT_MHZ);
}

static void pwm_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
    (void)RCC->APB1ENR;

    /* APB1 timer clock: PCLK1, doubled if APB1 is divided. */
    static const uint8_t apbShift[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    uint32_t pclk1 = SystemCoreClock >> apbShift[ppre1];
    timerClockHz = ppre1 >= 4u ? pclk1 * 2u : pclk1;

    for (unsigned ti = 0; ti < TIMER_COUNT; ti++) {
        TIM_TypeDef *t = pwmTimers[ti].tim;
        t->CR1 = 0;
        t->CCER = 0;
        t->CCMR1 = 0;
        t->CCMR2 = 0;
    }
    /* Each channel: PWM mode 1, no compare preload, output enabled. */
    for (unsigned i = 0; i < PWM_COUNT; i++) {
        TIM_TypeDef *t = pwmTimers[pwmChannels[i].timer].tim;
        uint32_t ch = pwmChannels[i].ch;
        volatile uint32_t *ccmr = ch <= 2u ? &t->CCMR1 : &t->CCMR2;
        uint32_t shift = ((ch - 1u) % 2u) * 8u;
        *ccmr = (*ccmr & ~(0xFFu << shift)) | (6u << (shift + 4u));
        t->CCER |= TIM_CCER_CC1E << (4u * (ch - 1u));
    }
    pwm_defaults();
    for (unsigned ti = 0; ti < TIMER_COUNT; ti++)
        pwmTimers[ti].tim->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

/* ---- Faults ---- */

/* One byte on PA9 by hand (interrupts off). */
static void fault_putc(char c)
{
    uint32_t bitCycles = (SystemCoreClock + BAUD / 2u) / BAUD;
    uint32_t t = DWT->CYCCNT;
    for (unsigned bit = 0; bit < 10; bit++) {
        pin_write(PIN_TX, swu_tx_level((uint8_t)c, bit));
        t += bitCycles;
        while ((int32_t)(DWT->CYCCNT - t) < 0) {
        }
    }
}

static void fault_stop(const char *what)
{
    __disable_irq();
    pin_write(PIN_E, !E_ON_LEVEL);      /* motor off before anything else */
    for (unsigned i = 0; i < PWM_COUNT; i++) {
        if (i == PWM_E)
            pin_output(pwmChannels[i].pin, !E_ON_LEVEL);
        else
            pin_input(pwmChannels[i].pin, 0);
    }

    TIM1->DIER = 0;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    pin_output(PIN_TX, true);           /* take PA9 from the timer, idle high */
    for (int i = 0; i < 2; i++)
        fault_putc('\xFF');             /* a little idle time (all ones) first */

    static const char head[] = "\r\n\r\n*** ";
    static const char tail[] = ": motor and PWM off, resetting ***\r\n";
    const char *parts[3] = {head, what, tail};
    for (int p = 0; p < 3; p++)
        for (const char *s = parts[p]; *s; s++)
            fault_putc(*s);
    NVIC_SystemReset();
}

void HardFault_Handler(void)  { fault_stop("HardFault"); }
void NMI_Handler(void)        { fault_stop("NMI"); }
void MemManage_Handler(void)  { fault_stop("MemManage fault"); }
void BusFault_Handler(void)   { fault_stop("BusFault"); }
void UsageFault_Handler(void) { fault_stop("UsageFault"); }

/* ---- Display ---- */

static void put_level(bool high)
{
    uart_puts(high ? "high" : "low");
}

/* Warns if a pin reads back different from what we're driving. */
static void put_pin_check(struct Pin p, const char *name)
{
    bool driven = pin_driven(p);
    if (pin_read(p) != driven) {
        uart_puts("  !! ");
        uart_puts(name);
        uart_puts(" set ");
        put_level(driven);
        uart_puts(" but reads ");
        put_level(!driven);
        uart_puts(": shorted or overloaded?");
    }
}

static bool limit_pressed(struct Pin p)
{
    return pin_read(p) == (LIMIT_ACTIVE_HIGH != 0);
}

static void put_drive(void)
{
    uint32_t key = irq_lock();          /* one consistent snapshot */
    enum DriveMode m = mode;
    uint32_t total = pulseMs, left = pulseLeftMs, pause = pauseLeftMs;
    bool waiting = pulseWaitTick;
    struct AutoRun run = autoRun;
    uint32_t cycles = autoSet.cycles;
    irq_unlock(key);

    if (m == MODE_OFF) {
        uart_puts("off");
    } else if (m == MODE_ON) {
        uart_puts("on");
    } else if (m == MODE_AUTO) {
        uart_puts("auto ");
        if (waiting) {
            uart_puts("starting");
        } else if (run.step == AUTO_PULSE) {
            uart_puts("pulse ");
            uart_put_u32(run.pulses + 1u);
            uart_puts(", ");
            uart_put_u32(run.leftMs);
            uart_puts(" ms left");
        } else if (run.step == AUTO_WAIT) {
            uart_puts("wait after pulse ");
            uart_put_u32(run.pulses);
            uart_puts(", ");
            uart_put_u32(run.leftMs);
            uart_puts(" ms left");
        } else {
            uart_puts("rest, ");
            uart_put_u32(run.leftMs);
            uart_puts(" ms to reversing");
        }
        uart_puts(", cycle ");
        uart_put_u32(run.ways / 2u + 1u);
        if (cycles) {
            uart_puts(" of ");
            uart_put_u32(cycles);
        }
    } else {
        uart_puts("pulse ");
        uart_put_u32(total);
        uart_puts(" ms, ");
        uart_put_u32(left);
        uart_puts(waiting ? " ms left (starting)" : " ms left");
    }
    if (m != MODE_OFF && pause > 0)
        uart_puts(" (paused for the direction change)");
}

/* One line, printed after every command. */
static void print_settings(void)
{
    uart_puts("  dir ");
    uart_putc(dirHigh ? '1' : '0');
    uart_puts(" | drive ");
    put_drive();
    uart_puts(" | E ");
    if (pwmOn[PWM_E])
        uart_puts("PWM test");
    else
        put_level(pin_driven(PIN_E));
    uart_puts(", M ");
    put_level(pin_driven(PIN_M));
    uart_puts(" | limits: max ");
    uart_puts(limit_pressed(PIN_LIMIT_MAX) ? "PRESSED" : "open");
    uart_puts(", min ");
    uart_puts(limit_pressed(PIN_LIMIT_MIN) ? "PRESSED" : "open");
    if (!pwmOn[PWM_E])
        put_pin_check(PIN_E, "E (PA0)");
    put_pin_check(PIN_M, "M (PE0)");
    uart_puts("\r\n");
}

static void print_status(void)
{
    uart_puts("direction  ");
    uart_putc(dirHigh ? '1' : '0');
    uart_puts("   M = PE0, J1 pin 4, ");
    put_level(pin_driven(PIN_M));
    uart_puts("\r\ndrive      ");
    put_drive();
    uart_puts("   E = PA0, J1 pin 5, ");
    if (pwmOn[PWM_E])
        uart_puts("PWM test");
    else
        put_level(pin_driven(PIN_E));
    uart_puts("\r\nlimits     EXT_MAX (PC2) ");
    uart_puts(limit_pressed(PIN_LIMIT_MAX) ? "PRESSED" : "open");
    uart_puts(", EXT_MIN (PC3) ");
    uart_puts(limit_pressed(PIN_LIMIT_MIN) ? "PRESSED" : "open");
    uart_puts("   (shown only; they don't stop the motor)");

    uint32_t now = msTicks;
    uart_puts("\r\nuptime     ");
    uart_put_u32(now / 1000u);
    uart_putc('.');
    uart_put_u32((now / 100u) % 10u);
    uart_puts(" s\r\nreset      ");
    uart_puts(resetCause);
    uart_puts("\r\nserial     ");
    uart_put_u32(serialErrors.framing);
    uart_puts(" framing, ");
    uart_put_u32(serialErrors.noise);
    uart_puts(" noise errors, ");
    uart_put_u32(serialErrors.lost);
    uart_puts(" bytes lost (buffer full)\r\n");

    bool bad = false;
    if ((!pwmOn[PWM_E] && pin_read(PIN_E) != pin_driven(PIN_E)) || pin_read(PIN_M) != pin_driven(PIN_M))
        bad = true;
    if (bad) {
        uart_puts("pins      ");
        if (!pwmOn[PWM_E])
            put_pin_check(PIN_E, "E (PA0)");
        put_pin_check(PIN_M, "M (PE0)");
        uart_puts("\r\n");
    }
}

static void print_help(void)
{
    uart_puts("Commands:\r\n"
              "  dir 0 | 1 | toggle   direction pin M (PE0, J1 pin 4) low or high\r\n"
              "  drive on | off       E (PA0, J1 pin 5) on at 100 %, or off\r\n"
              "  pulse <ms>           E on for <ms> milliseconds (1 to 60000), then off\r\n"
              "  stop                 drive off (automatic mode too)\r\n"
              "  status               pins, limit switches, uptime, reset cause, serial errors\r\n"
              "  help\r\n"
              "Automatic: pulses one way, rests, reverses, pulses back, rests, reverses...\r\n"
              "  auto                 its settings, and what it would do\r\n"
              "  auto start | stop    (stop, Esc, drive and pulse stop it too)\r\n"
              "  auto <name> <value>...  any number at once; times in ms, or s (1.5s):\r\n"
              "    pulse <ms>         E on, each pulse\r\n"
              "    wait <ms>          E off after each pulse (0: pulses run together)\r\n"
              "    timeout <ms>       reverse once this long has gone by one way (0: never)\r\n"
              "    count <n>          or after this many pulses one way (0: no limit)\r\n"
              "    rest <ms>          E off before each reversal (100 or more)\r\n"
              "    cycles <n>         there-and-back cycles, then stop (0: until stopped)\r\n"
              "    first 0 | 1        direction it starts in\r\n"
              "  auto default         settings back to the defaults\r\n"
              "  e.g. auto pulse 200 wait 800 timeout 10s rest 1s cycles 3 start\r\n"
              "PWM test (<pin>: j1.3 j1.5 j1.6 j2.3 j2.5 j2.8 j7.4, or pa6 etc.):\r\n"
              "  pwm                  list pins and timers, and measure each pin\r\n"
              "  pwm on | off         all pins on / off (not j1.5, the motor's E)\r\n"
              "  pwm default          all off, 1 kHz, 50 %, normal polarity\r\n"
              "  pwm <pin> on | off\r\n"
              "  pwm <pin> duty <%>   e.g. 37.5         (or all / tim2 / tim3 / tim4)\r\n"
              "  pwm <pin> pol normal | inverted        (or all / tim2 / tim3 / tim4)\r\n"
              "  pwm <pin> ccr <n>    raw compare; n = arr + 1 is 100 %\r\n"
              "  pwm <tim> freq <Hz>  e.g. 20k, 0.5, 2M (or a pin: sets its timer; or all)\r\n"
              "  pwm <tim> psc <n>    raw prescaler     (or a pin: sets its timer)\r\n"
              "  pwm <tim> arr <n>    raw auto-reload   (or a pin: sets its timer)\r\n"
              "Servo (ST3215 on J7 pin 3; 1 kohm from it to 3.3 V):\r\n"
              "  servo echo           check the line by itself: our bytes should come back\r\n"
              "  servo scan           find servos (ids 0-253)\r\n"
              "  servo id [n]         which servo to talk to (default 1)\r\n"
              "  servo ping | status  is it there; position, supply, load, faults...\r\n"
              "  servo angle <deg> [speed]   move to 0-360 deg (180 = middle)\r\n"
              "  servo pos <steps> [speed]   move to 0-4095 (2048 = middle)\r\n"
              "  servo speed <steps/s> | acc <n>   for moves (default 500, 50)\r\n"
              "  servo torque on | off\r\n"
              "  servo stop           hold where it is now (Esc does too)\r\n"
              "  servo read <addr> [len] | write <addr> <byte>...   raw registers\r\n"
              "Esc or Ctrl-C turns the drive and all PWM off (and holds the servo) at once.\r\n"
              "Up / Down arrows: previous commands.\r\n"
              "LEDs on J5: pin 3 heartbeat, 4 drive, 5 direction, 6 serial.\r\n");
}

/* ---- Line input ---- */

static char line[LINE_MAX];
static size_t lineLen;
static bool lineOverflow;

static void prompt(void)
{
    uart_puts("> ");
}

/* For messages that aren't replies: own line, then the prompt comes back. */
static void async_begin(void)
{
    uart_puts("\r\n");
}

static void async_end(void)
{
    prompt();
    uart_write(line, lineLen);
}

/* History (Up / Down arrows) */

static char history[HISTORY_SIZE][LINE_MAX];
static size_t historyLen[HISTORY_SIZE];
static uint32_t historyTotal;           /* commands ever stored */
static int browse = -1;                 /* -1: new line; 0: newest shown; 1: the one before... */
static char draft[LINE_MAX];            /* the new line, kept while browsing */
static size_t draftLen;

static uint32_t history_count(void)
{
    return historyTotal < HISTORY_SIZE ? historyTotal : HISTORY_SIZE;
}

static uint32_t history_slot(uint32_t back)     /* back = 0: newest */
{
    return (historyTotal - 1u - back) % HISTORY_SIZE;
}

/* Stores a command (not blank lines or repeats). */
static void history_add(const char *s, size_t len)
{
    size_t i = 0;
    while (i < len && (s[i] == ' ' || s[i] == '\t'))
        i++;
    if (i == len)
        return;
    if (historyTotal > 0) {
        uint32_t newest = history_slot(0);
        if (historyLen[newest] == len && memcmp(history[newest], s, len) == 0)
            return;
    }
    uint32_t slot = historyTotal % HISTORY_SIZE;
    memcpy(history[slot], s, len);
    historyLen[slot] = len;
    historyTotal++;
}

/* Replaces the line being typed, on screen too. */
static void line_replace(const char *s, size_t len)
{
    memcpy(line, s, len);
    lineLen = len;
    lineOverflow = false;
    uart_puts("\r\x1b[K");             /* start of the line, erase it */
    prompt();
    uart_write(line, lineLen);
}

static void history_up(void)
{
    if ((uint32_t)(browse + 1) >= history_count())
        return;                         /* nothing older */
    if (browse < 0) {
        memcpy(draft, line, lineLen);
        draftLen = lineLen;
    }
    browse++;
    uint32_t slot = history_slot((uint32_t)browse);
    line_replace(history[slot], historyLen[slot]);
}

static void history_down(void)
{
    if (browse < 0)
        return;
    browse--;
    if (browse < 0) {
        line_replace(draft, draftLen);
    } else {
        uint32_t slot = history_slot((uint32_t)browse);
        line_replace(history[slot], historyLen[slot]);
    }
}

static char lower(char c)
{
    return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

/* Case-insensitive compare of a whole word. */
static bool word_is(const char *s, size_t len, const char *word)
{
    size_t i = 0;
    for (; i < len && word[i]; i++)
        if (lower(s[i]) != word[i])
            return false;
    return i == len && word[i] == '\0';
}

/* Next space-separated word. False if there isn't one. */
static bool next_word(const char **s, size_t *len, const char **word, size_t *wordLen)
{
    while (*len && (**s == ' ' || **s == '\t')) {
        (*s)++;
        (*len)--;
    }
    if (*len == 0)
        return false;
    *word = *s;
    while (*len && **s != ' ' && **s != '\t') {
        (*s)++;
        (*len)--;
    }
    *wordLen = (size_t)(*s - *word);
    return true;
}

/* Whole word must be digits, 1 to max. */
static bool parse_u32(const char *s, size_t len, uint32_t max, uint32_t *value)
{
    if (len == 0 || len > 9)
        return false;
    uint32_t v = 0;
    for (size_t i = 0; i < len; i++) {
        if (s[i] < '0' || s[i] > '9')
            return false;
        v = v * 10u + (uint32_t)(s[i] - '0');
    }
    if (v < 1 || v > max)
        return false;
    *value = v;
    return true;
}

/* ---- Commands ---- */

static void cmd_dir(const char *args, size_t len)
{
    const char *w, *extra;
    size_t wl, el;

    if (!next_word(&args, &len, &w, &wl)) {
        print_settings();
        return;
    }
    if (next_word(&args, &len, &extra, &el)) {
        uart_puts("usage: dir 0 | 1 | toggle\r\n");
        return;
    }
    if (mode == MODE_AUTO) {
        uart_puts("automatic mode is running and sets the direction: auto stop first\r\n");
        return;
    }

    bool high;
    if (word_is(w, wl, "0") || word_is(w, wl, "low"))
        high = false;
    else if (word_is(w, wl, "1") || word_is(w, wl, "high"))
        high = true;
    else if (word_is(w, wl, "toggle"))
        high = !dirHigh;
    else {
        uart_puts("usage: dir 0 | 1 | toggle\r\n");
        return;
    }

    if (drive_set_dir(high)) {
        uart_puts("E off for ");
        uart_put_u32(REVERSE_PAUSE_MS);
        uart_puts(" ms while the direction changes\r\n");
    }
    print_settings();
}

static void cmd_drive(const char *args, size_t len)
{
    const char *w, *extra;
    size_t wl, el;

    if (!next_word(&args, &len, &w, &wl)) {
        print_settings();
        return;
    }
    if (next_word(&args, &len, &extra, &el)) {
        uart_puts("usage: drive on | off\r\n");
        return;
    }
    bool wasAuto = mode == MODE_AUTO;
    if (word_is(w, wl, "on"))
        drive_set(true);
    else if (word_is(w, wl, "off"))
        drive_set(false);
    else {
        uart_puts("usage: drive on | off\r\n");
        return;
    }
    if (wasAuto)
        uart_puts("automatic mode stopped\r\n");
    print_settings();
}

static void cmd_pulse(const char *args, size_t len)
{
    const char *w, *extra;
    size_t wl, el;
    uint32_t ms;

    if (!next_word(&args, &len, &w, &wl) || next_word(&args, &len, &extra, &el) ||
        !parse_u32(w, wl, PULSE_MAX_MS, &ms)) {
        uart_puts("usage: pulse <ms>, 1 to ");
        uart_put_u32(PULSE_MAX_MS);
        uart_puts("\r\n");
        return;
    }
    bool wasAuto = mode == MODE_AUTO;
    drive_pulse(ms);
    if (wasAuto)
        uart_puts("automatic mode stopped\r\n");
    print_settings();
}

/* PWM test commands */

/* Text buffer, for lining up columns. */
struct Text {
    char s[120];
    size_t n;
};

static void t_putc(struct Text *t, char c)
{
    if (t->n < sizeof t->s - 1u)
        t->s[t->n++] = c;
    t->s[t->n] = '\0';
}

static void t_puts(struct Text *t, const char *s)
{
    while (*s)
        t_putc(t, *s++);
}

static void t_u64(struct Text *t, uint64_t v)
{
    char digits[20];
    int n = 0;
    do {
        digits[n++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v);
    while (n)
        t_putc(t, digits[--n]);
}

/* v in thousandths, up to 3 decimals. */
static void t_milli(struct Text *t, uint64_t v)
{
    t_u64(t, v / 1000u);
    uint32_t frac = (uint32_t)(v % 1000u);
    if (frac) {
        t_putc(t, '.');
        char d[3] = {(char)('0' + frac / 100u), (char)('0' + frac / 10u % 10u), (char)('0' + frac % 10u)};
        int last = d[2] != '0' ? 3 : d[1] != '0' ? 2 : 1;
        for (int i = 0; i < last; i++)
            t_putc(t, d[i]);
    }
}

static void t_pad(struct Text *t, size_t column)
{
    while (t->n < column)
        t_putc(t, ' ');
}

/* Case-insensitive whole-word compare. */
static bool word_is_name(const char *s, size_t len, const char *name)
{
    size_t i = 0;
    for (; i < len && name[i]; i++)
        if (lower(s[i]) != lower(name[i]))
            return false;
    return i == len && name[i] == '\0';
}

static int pwm_find_pin(const char *w, size_t wl)
{
    for (unsigned i = 0; i < PWM_COUNT; i++)
        if (word_is_name(w, wl, pwmChannels[i].name) || word_is_name(w, wl, pwmChannels[i].chip) ||
            word_is_name(w, wl, pwmChannels[i].net))
            return (int)i;
    return -1;
}

static int pwm_find_timer(const char *w, size_t wl)
{
    for (unsigned ti = 0; ti < TIMER_COUNT; ti++)
        if (word_is_name(w, wl, pwmTimers[ti].name))
            return (int)ti;
    return -1;
}

/* Number with up to 3 decimals and k / M / % suffix, in thousandths. */
static bool parse_milli(const char *s, size_t len, uint64_t *out)
{
    uint64_t mult = 1;
    if (len > 0 && (s[len - 1] == 'k' || s[len - 1] == 'K')) {
        mult = 1000u;
        len--;
    } else if (len > 0 && s[len - 1] == 'M') {
        mult = 1000000u;
        len--;
    } else if (len > 0 && s[len - 1] == '%') {
        len--;
    }
    uint64_t whole = 0, frac = 0;
    int fracDigits = 0;
    bool dot = false, digits = false;
    for (size_t i = 0; i < len; i++) {
        if (s[i] == '.' && !dot) {
            dot = true;
            continue;
        }
        if (s[i] < '0' || s[i] > '9')
            return false;
        digits = true;
        uint32_t d = (uint32_t)(s[i] - '0');
        if (!dot) {
            whole = whole * 10u + d;
            if (whole > 1000000000u)
                return false;
        } else if (fracDigits < 3) {
            frac = frac * 10u + d;
            fracDigits++;
        }
    }
    if (!digits)
        return false;
    for (; fracDigits < 3; fracDigits++)
        frac *= 10u;
    *out = (whole * 1000u + frac) * mult;
    return true;
}

static bool parse_u64(const char *s, size_t len, uint64_t max, uint64_t *out)
{
    if (len == 0 || len > 10)
        return false;
    uint64_t v = 0;
    for (size_t i = 0; i < len; i++) {
        if (s[i] < '0' || s[i] > '9')
            return false;
        v = v * 10u + (uint64_t)(s[i] - '0');
    }
    if (v > max)
        return false;
    *out = v;
    return true;
}

/* How much of the time each pin that's on reads high. Returns samples. */
static uint32_t pwm_measure(uint32_t high[PWM_COUNT])
{
    bool any = false;
    for (unsigned i = 0; i < PWM_COUNT; i++) {
        high[i] = 0;
        any = any || pwmOn[i];
    }
    if (!any)
        return 0;

    uint32_t samples = 0, start = msTicks;
    while (msTicks - start < PWM_MEASURE_MS) {
        IWDG->KR = 0xAAAA;
        for (unsigned i = 0; i < PWM_COUNT; i++)
            if (pwmOn[i] && pin_read(pwmChannels[i].pin))
                high[i]++;
        samples++;
    }
    return samples;
}

static void pwm_list(void)
{
    uint32_t high[PWM_COUNT];
    uint32_t samples = pwm_measure(high);
    struct Text t;

    uart_puts("pin   chip net        timer     on   duty      ccr          pol  pin reads\r\n");
    for (unsigned i = 0; i < PWM_COUNT; i++) {
        unsigned ti = pwmChannels[i].timer;
        uint64_t period = pwm_period(ti);
        uint32_t ccr = *pwm_ccr(i);
        t.n = 0;
        t.s[0] = '\0';
        t_puts(&t, pwmChannels[i].name);
        t_pad(&t, 6);
        t_puts(&t, pwmChannels[i].chip);
        t_pad(&t, 11);
        t_puts(&t, pwmChannels[i].net);
        t_pad(&t, 22);
        t_puts(&t, pwmTimers[ti].name);
        t_puts(&t, " ch");
        t_putc(&t, (char)('0' + pwmChannels[i].ch));
        t_pad(&t, 32);
        t_puts(&t, pwmOn[i] ? "ON" : "off");
        t_pad(&t, 37);
        t_milli(&t, (ccr >= period ? 100000u : (uint64_t)ccr * 100000u / period));
        t_puts(&t, " %");
        t_pad(&t, 47);
        t_u64(&t, ccr);
        t_pad(&t, 60);
        t_puts(&t, pwmInverted[i] ? "inv" : "norm");
        t_pad(&t, 65);
        if (pwmOn[i] && samples) {
            t_milli(&t, (uint64_t)high[i] * 100000u / samples / 100u * 100u);
            t_puts(&t, " %");
            if (pwm_timer_milliHz(ti) < 10000u)
                t_puts(&t, " (slow)");
        } else {
            t_puts(&t, "-");
        }
        uart_puts(t.s);
        if (i == PWM_E)
            uart_puts("  motor E");
        uart_puts("\r\n");
    }

    uart_puts("timer freq             psc    arr          pins\r\n");
    for (unsigned ti = 0; ti < TIMER_COUNT; ti++) {
        t.n = 0;
        t.s[0] = '\0';
        t_puts(&t, pwmTimers[ti].name);
        t_pad(&t, 6);
        t_milli(&t, pwm_timer_milliHz(ti));
        t_puts(&t, " Hz");
        t_pad(&t, 23);
        t_u64(&t, pwmTimers[ti].tim->PSC);
        t_pad(&t, 30);
        t_u64(&t, pwmTimers[ti].tim->ARR);
        t_pad(&t, 43);
        for (unsigned i = 0; i < PWM_COUNT; i++) {
            if (pwmChannels[i].timer == ti) {
                t_puts(&t, pwmChannels[i].name);
                t_putc(&t, ' ');
            }
        }
        uart_puts(t.s);
        uart_puts(pwmTimers[ti].is32 ? " (32-bit)\r\n" : "\r\n");
    }
    uart_puts("(timers count at ");
    uart_put_u32(timerClockHz / 1000000u);
    uart_puts(" MHz. 'pin reads': how much of the time the pin itself is high)\r\n");
}

static void pwm_usage(void)
{
    uart_puts("usage: pwm | pwm on | off | default | pwm <pin|all|tim> on | off | duty <%> |\r\n"
              "       pol normal|inverted | ccr <n> | freq <Hz> | psc <n> | arr <n>   (see help)\r\n");
}

static void pwm_set_on(unsigned i, bool on)
{
    if (on == pwmOn[i])
        return;
    if (!on) {
        pwm_pin_off(i);
        return;
    }
    if (i == PWM_E) {
        drive_set(false);               /* E belongs to the PWM test until a drive command */
        uart_puts("j1.5 is the motor's E: drive off, the PWM test drives it now\r\n");
    }
    pwm_pin_on(i);
}

/* pwm [<target> <action> [<value>]] */
static void cmd_pwm(const char *args, size_t len)
{
    const char *w[4];
    size_t wl[4];
    unsigned n = 0;
    while (n < 4 && next_word(&args, &len, &w[n], &wl[n]))
        n++;
    if (n == 4) {
        pwm_usage();
        return;
    }

    if (n == 0 || (n == 1 && word_is(w[0], wl[0], "list"))) {
        pwm_list();
        return;
    }
    if (n == 1) {
        if (word_is(w[0], wl[0], "on")) {
            for (unsigned i = 0; i < PWM_COUNT; i++)
                if (pwmChannels[i].inAll)
                    pwm_set_on(i, true);
        } else if (word_is(w[0], wl[0], "off")) {
            pwm_all_off();
        } else if (word_is(w[0], wl[0], "default")) {
            pwm_defaults();
        } else {
            pwm_usage();
            return;
        }
        pwm_list();
        return;
    }

    /* Target pins and timers. 'all' and timers don't switch the motor's E. */
    bool pins[PWM_COUNT] = {false};
    bool tims[TIMER_COUNT] = {false};
    int pin = -1, tim = -1;
    if (word_is(w[0], wl[0], "all")) {
        for (unsigned i = 0; i < PWM_COUNT; i++)
            pins[i] = true;
        for (unsigned ti = 0; ti < TIMER_COUNT; ti++)
            tims[ti] = true;
    } else if ((pin = pwm_find_pin(w[0], wl[0])) >= 0) {
        pins[pin] = true;
        tims[pwmChannels[pin].timer] = true;
    } else if ((tim = pwm_find_timer(w[0], wl[0])) >= 0) {
        tims[tim] = true;
        for (unsigned i = 0; i < PWM_COUNT; i++)
            pins[i] = pwmChannels[i].timer == (unsigned)tim;
    } else {
        uart_puts("unknown pin or timer \"");
        uart_write(w[0], wl[0]);
        uart_puts("\" (type pwm for the list)\r\n");
        return;
    }

    const char *act = w[1], *val = n == 3 ? w[2] : NULL;
    size_t actLen = wl[1], valLen = n == 3 ? wl[2] : 0;
    uint64_t v;

    if (word_is(act, actLen, "on") || word_is(act, actLen, "off")) {
        if (val) {
            pwm_usage();
            return;
        }
        bool on = word_is(act, actLen, "on");
        for (unsigned i = 0; i < PWM_COUNT; i++)
            if (pins[i] && (pwmChannels[i].inAll || pin >= 0))
                pwm_set_on(i, on);
    } else if (word_is(act, actLen, "duty")) {
        if (!val || !parse_milli(val, valLen, &v) || v > 100000u) {
            uart_puts("duty: 0 to 100 %, up to 3 decimals\r\n");
            return;
        }
        for (unsigned i = 0; i < PWM_COUNT; i++) {
            if (pins[i]) {
                pwmDuty[i] = (uint32_t)v;
                pwm_apply_duty(i);
            }
        }
    } else if (word_is(act, actLen, "pol")) {
        bool inv;
        if (val && word_is(val, valLen, "normal"))
            inv = false;
        else if (val && (word_is(val, valLen, "inverted") || word_is(val, valLen, "inv")))
            inv = true;
        else {
            uart_puts("pol: normal or inverted\r\n");
            return;
        }
        for (unsigned i = 0; i < PWM_COUNT; i++) {
            if (pins[i]) {
                pwmInverted[i] = inv;
                pwm_apply_polarity(i);
            }
        }
    } else if (word_is(act, actLen, "ccr")) {
        if (pin < 0) {
            uart_puts("ccr: give one pin\r\n");
            return;
        }
        uint64_t period = pwm_period(pwmChannels[pin].timer);
        if (!val || !parse_u64(val, valLen, period, &v)) {
            uart_puts("ccr: 0 to ");
            uart_put_u64(period);
            uart_puts(" (arr + 1 = 100 %)\r\n");
            return;
        }
        *pwm_ccr((unsigned)pin) = (uint32_t)(v > 0xFFFFFFFFu ? 0xFFFFFFFFu : v);
        pwmDuty[pin] = (uint32_t)(v * 100000u / period);    /* kept if the frequency changes */
    } else if (word_is(act, actLen, "freq")) {
        if (!val || !parse_milli(val, valLen, &v) || v == 0) {
            uart_puts("freq: in Hz, e.g. 1000, 20k, 0.5, 2M\r\n");
            return;
        }
        for (unsigned ti = 0; ti < TIMER_COUNT; ti++) {
            if (tims[ti] && !pwm_timer_freq(ti, v)) {
                uart_puts(pwmTimers[ti].name);
                uart_puts(": can't do that frequency (about ");
                uart_puts(pwmTimers[ti].is32 ? "0.001" : "0.004");
                uart_puts(" Hz to ");
                uart_put_u32(timerClockHz / 2u);
                uart_puts(" Hz)\r\n");
            }
        }
        if (pin >= 0) {
            uart_puts("(sets ");
            uart_puts(pwmTimers[pwmChannels[pin].timer].name);
            uart_puts(", so every pin on it)\r\n");
        }
    } else if (word_is(act, actLen, "psc") || word_is(act, actLen, "arr")) {
        if (word_is(w[0], wl[0], "all")) {
            uart_puts("psc / arr: give one timer or pin\r\n");
            return;
        }
        unsigned ti = pin >= 0 ? pwmChannels[pin].timer : (unsigned)tim;
        bool isPsc = word_is(act, actLen, "psc");
        uint64_t max = isPsc ? 0xFFFFu : (pwmTimers[ti].is32 ? 0xFFFFFFFFu : 0xFFFFu);
        if (!val || !parse_u64(val, valLen, max, &v) || (!isPsc && v == 0)) {
            uart_puts(isPsc ? "psc: 0 to " : "arr: 1 to ");
            uart_put_u64(max);
            uart_puts("\r\n");
            return;
        }
        if (isPsc)
            pwm_timer_set(ti, (uint32_t)v, pwmTimers[ti].tim->ARR);
        else
            pwm_timer_set(ti, pwmTimers[ti].tim->PSC, (uint32_t)v);
    } else {
        pwm_usage();
        return;
    }
    pwm_list();
}

/* ---- Servo bus ----
 * ST3215 on J7 pin 3 (PB6): USART1 half-duplex, 1 Mbaud, DMA both ways.
 * Our own request echoes back first; it's checked and skipped. */

#define REPLY_GAP_US        100u        /* line quiet this long: reply finished */

static uint8_t servoTx[80];
static uint8_t servoRx[160];            /* our echo, then the reply */
static bool servoEchoOk;                /* last request echoed back exactly */
static uint32_t servoEchoLen;           /* echo bytes received */

static uint32_t us_to_cycles(uint32_t us)
{
    return us * (SystemCoreClock / 1000000u);
}

void servo_port_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     /* cycle counter, for timing */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    static const uint8_t apbShift[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    uint32_t pclk2 = SystemCoreClock >> apbShift[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos];
    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = USART_CR3_HDSEL | USART_CR3_DMAR | USART_CR3_DMAT;
    USART1->BRR = (pclk2 + SERVO_BAUD / 2u) / SERVO_BAUD;      /* 16 MHz / 1 M = 16 */
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

    pin_alternate(PIN_SERVO, 7, 1);     /* AF7, pull-up */
    PIN_SERVO.port->OTYPER |= 1u << PIN_SERVO.n;                /* open-drain */
    PIN_SERVO.port->OSPEEDR |= 3u << (2u * PIN_SERVO.n);

    DMA2_Stream2->CR = 0;
    DMA2_Stream7->CR = 0;
    while ((DMA2_Stream2->CR | DMA2_Stream7->CR) & DMA_SxCR_EN) {
    }
    DMA2_Stream2->PAR = (uint32_t)&USART1->DR;
    DMA2_Stream7->PAR = (uint32_t)&USART1->DR;
}

static void servo_dma_stop(void)
{
    DMA2_Stream2->CR &= ~DMA_SxCR_EN;
    DMA2_Stream7->CR &= ~DMA_SxCR_EN;
    while ((DMA2_Stream2->CR | DMA2_Stream7->CR) & DMA_SxCR_EN) {
    }
    DMA2->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 |
                  DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2;
    DMA2->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 |
                  DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7;
    (void)USART1->SR;                   /* clear error flags */
    (void)USART1->DR;
}

static uint32_t servo_received(void)
{
    return sizeof servoRx - DMA2_Stream2->NDTR;
}

uint32_t servo_port_exchange(const uint8_t *tx, uint32_t txLen, uint8_t *rx, uint32_t rxMax,
                             uint32_t timeoutUs)
{
    if (txLen > sizeof servoTx)
        txLen = sizeof servoTx;
    servo_dma_stop();
    memcpy(servoTx, tx, txLen);

    DMA2_Stream2->M0AR = (uint32_t)servoRx;
    DMA2_Stream2->NDTR = sizeof servoRx;
    DMA2_Stream2->CR = (4u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_EN;

    USART1->SR = ~USART_SR_TC;
    DMA2_Stream7->M0AR = (uint32_t)servoTx;
    DMA2_Stream7->NDTR = txLen;
    DMA2_Stream7->CR = (4u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_EN;

    /* wait for the request to go out */
    uint32_t start = DWT->CYCCNT;
    while (!(DMA2_Stream7->NDTR == 0 && (USART1->SR & USART_SR_TC)) &&
           DWT->CYCCNT - start < us_to_cycles(txLen * 20u + 200u)) {
    }

    /* our own request, heard back */
    start = DWT->CYCCNT;
    while (servo_received() < txLen && DWT->CYCCNT - start < us_to_cycles(50)) {
    }
    uint32_t got = servo_received();
    servoEchoLen = got < txLen ? got : txLen;
    servoEchoOk = got >= txLen && memcmp(servoRx, servoTx, txLen) == 0;
    uint32_t skip = servoEchoLen;

    /* reply: wait for the first byte, then for a quiet line */
    start = DWT->CYCCNT;
    uint32_t lastChange = start, count = servo_received();
    for (;;) {
        uint32_t now = DWT->CYCCNT, c = servo_received();
        if (c != count) {
            count = c;
            lastChange = now;
        }
        if (count >= sizeof servoRx)
            break;
        if (count <= skip && now - start >= us_to_cycles(timeoutUs))
            break;
        if (count > skip && now - lastChange >= us_to_cycles(REPLY_GAP_US))
            break;
    }
    servo_dma_stop();

    uint32_t n = count > skip ? count - skip : 0;
    if (n > rxMax)
        n = rxMax;
    memcpy(rx, servoRx + skip, n);
    return n;
}

/* Servo commands */

static uint8_t servoId = 1;
static uint32_t servoSpeed = SERVO_SPEED, servoAcc = SERVO_ACC;
static bool servoUsed;                  /* moved since reset: Esc holds it too */

static void out(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static void out(const char *fmt, ...)
{
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0)
        uart_write(buf, (size_t)n < sizeof buf ? (size_t)n : sizeof buf - 1u);
}

/* Reports what went wrong with an exchange. True if it worked. */
static bool servo_report(enum ServoResult r, uint8_t error)
{
    if (r == SERVO_NO_REPLY) {
        out("no reply from servo %u", servoId);
        if (!servoEchoOk)
            out(": our own request didn't come back either (%lu of it did), so check the\r\n"
                "line: wiring to J7 pin 3, the 1 kohm pull-up to 3.3 V\r\n", (unsigned long)servoEchoLen);
        else
            out(" (the line itself is fine: check the servo's 12 V, GND and id)\r\n");
        return false;
    }
    if (r == SERVO_BAD_REPLY) {
        out("bad reply from servo %u (%lu bytes:", servoId, (unsigned long)servoLastRxLen);
        for (uint32_t i = 0; i < servoLastRxLen && i < 20u; i++)
            out(" %02X", servoLastRx[i]);
        out(")\r\n");
        return false;
    }
    if (error)
        out("servo %u reports fault flags 0x%02X\r\n", servoId, error);
    return true;
}

static bool servo_get(uint8_t addr, uint8_t len, uint8_t *data)
{
    uint8_t err = 0;
    enum ServoResult r = servo_read(servoId, addr, len, data, &err);
    return servo_report(r, err);
}

static bool servo_set(uint8_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t err = 0;
    enum ServoResult r = servo_write(servoId, addr, data, len, &err);
    return servo_report(r, err);
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static int32_t sign_mag(uint16_t v, unsigned signBit)   /* the given bit is the sign */
{
    int32_t mag = (int32_t)(v & ((1u << signBit) - 1u));
    return (v & (1u << signBit)) ? -mag : mag;
}

static void out_tenths(const char *before, int32_t tenths, const char *after)
{
    const char *sign = tenths < 0 ? "-" : "";
    if (tenths < 0)
        tenths = -tenths;
    out("%s%s%ld.%ld%s", before, sign, (long)(tenths / 10), (long)(tenths % 10), after);
}

static int32_t steps_to_decideg(int32_t steps)
{
    return steps * 3600 / 4096;
}

/* Torque on first (turning it on resets the goal), then the goal. */
static void servo_move(uint32_t steps, uint32_t speed)
{
    if (steps > 4095u)
        steps = 4095u;
    uint8_t torque;
    if (!servo_get(REG_TORQUE, 1, &torque))
        return;
    uint8_t on = 1;
    if (torque != 1 && !servo_set(REG_TORQUE, &on, 1))
        return;
    uint8_t goal[7] = {
        (uint8_t)servoAcc,                              /* 41 acceleration */
        (uint8_t)steps, (uint8_t)(steps >> 8),          /* 42 goal position */
        0, 0,                                           /* 44 goal time: unused */
        (uint8_t)speed, (uint8_t)(speed >> 8),          /* 46 goal speed */
    };
    if (!servo_set(REG_ACC, goal, sizeof goal))
        return;
    servoUsed = true;
    out("servo moving to %lu", (unsigned long)steps);
    out_tenths(" (", steps_to_decideg((int32_t)steps), " deg)");
    out(" at %lu steps/s, acc %lu\r\n", (unsigned long)speed, (unsigned long)servoAcc);
}

static void servo_hold(void)                /* goal = where it is now */
{
    uint8_t p[2];
    if (!servo_get(REG_PRESENT_POS, 2, p))
        return;
    int32_t pos = sign_mag(get16(p), 15);
    if (pos < 0)
        pos = 0;
    uint8_t goal[2] = {(uint8_t)pos, (uint8_t)(pos >> 8)};
    if (servo_set(REG_GOAL_POS, goal, 2))
        out("servo holding at %ld\r\n", (long)pos);
}

static void servo_echo_test(void)
{
    static const uint8_t pattern[6] = {0x55, 0xAA, 0x00, 0x0F, 0xF0, 0x55};    /* not a valid packet */
    uint8_t ignore[8];
    servo_port_exchange(pattern, sizeof pattern, ignore, sizeof ignore, 500);
    if (servoEchoOk) {
        out("line OK: all %u bytes came back as sent (J7 pin 3, USART1 and the pull-up work)\r\n",
            (unsigned)sizeof pattern);
        return;
    }
    out("line problem: %lu of %u bytes came back", (unsigned long)servoEchoLen, (unsigned)sizeof pattern);
    if (servoEchoLen) {
        out(", as");
        for (uint32_t i = 0; i < servoEchoLen; i++)
            out(" %02X", servoRx[i]);
        out(" (sent 55 AA 00 0F F0 55)");
    }
    out(".\r\nNothing back usually means no pull-up (1 kohm from J7 pin 3 to 3.3 V) or the\r\n"
        "line is held low.\r\n");
}

static void servo_scan(void)
{
    out("scanning ids 0-253...\r\n");
    unsigned found = 0;
    for (unsigned id = 0; id <= 253u; id++) {
        IWDG->KR = 0xAAAA;
        uint8_t err = 0, m[2], err2 = 0;
        if (servo_ping((uint8_t)id, 1000, &err) != SERVO_OK)
            continue;
        found++;
        if (servo_read((uint8_t)id, REG_MODEL, 2, m, &err2) == SERVO_OK)
            out("  id %u: model %u%s\r\n", id, get16(m), err ? " (fault flags set)" : "");
        else
            out("  id %u: answers ping, model unreadable\r\n", id);
    }
    if (found)
        out("%u found\r\n", found);
    else if (!servoEchoOk)
        out("none found, and our own bytes don't come back: check the line (servo echo)\r\n");
    else
        out("none found (the line works: check the servo's 12 V and GND)\r\n");
}

static void servo_status(void)
{
    uint8_t a[48], b[15];
    if (!servo_get(0, sizeof a, a) || !servo_get(REG_PRESENT_POS, sizeof b, b))
        return;
    int32_t pos = sign_mag(get16(&b[0]), 15);
    int32_t speed = sign_mag(get16(&b[2]), 15);
    int32_t load = sign_mag(get16(&b[4]), 10);
    int32_t current = sign_mag(get16(&b[13]), 15);
    uint16_t goal = get16(&a[REG_GOAL_POS]);
    static const char *const modes[4] = {"position", "wheel", "PWM", "step"};
    uint8_t servoMode = a[REG_MODE];

    out("servo %u, model %u\r\n", servoId, get16(&a[REG_MODEL]));
    out("  position  %ld", (long)pos);
    out_tenths(" (", steps_to_decideg(pos), " deg)");
    out(", goal %u", goal);
    out_tenths(" (", steps_to_decideg(goal), " deg)");
    out(", speed %ld steps/s, %s\r\n", (long)speed, b[10] ? "moving" : "not moving");
    out_tenths("  supply    ", b[6], " V");
    out(", %u C", b[7]);
    out_tenths(", load ", load, " %");
    out(", current %ld mA\r\n", (long)(current * 65 / 10));
    out("  torque    %s, mode %u (%s), limits %u to %u, offset %ld, fault flags 0x%02X\r\n",
        a[REG_TORQUE] ? "on" : "off", servoMode, servoMode < 4u ? modes[servoMode] : "?",
        get16(&a[REG_MIN_ANGLE]), get16(&a[REG_MAX_ANGLE]),
        (long)sign_mag(get16(&a[REG_OFFSET]), 11), b[9]);
}

/* Decimal, or hex with 0x. */
static bool parse_num(const char *s, size_t len, uint32_t max, uint32_t *value)
{
    uint32_t base = 10, v = 0;
    if (len > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        s += 2;
        len -= 2;
    }
    if (len == 0 || len > 8)
        return false;
    for (size_t i = 0; i < len; i++) {
        char c = lower(s[i]);
        uint32_t d;
        if (c >= '0' && c <= '9')
            d = (uint32_t)(c - '0');
        else if (base == 16 && c >= 'a' && c <= 'f')
            d = (uint32_t)(c - 'a' + 10);
        else
            return false;
        v = v * base + d;
    }
    if (v > max)
        return false;
    *value = v;
    return true;
}

static void cmd_servo(const char *args, size_t len)
{
    const char *sub, *w;
    size_t subLen, wl;
    uint32_t v, speed;
    uint8_t err = 0;

    if (!next_word(&args, &len, &sub, &subLen)) {
        out("usage: servo echo | scan | id | ping | status | angle | pos | speed | acc |\r\n"
            "       torque | stop | read | write   (see help)\r\n");
        return;
    }

    if (word_is(sub, subLen, "echo")) {
        servo_echo_test();
    } else if (word_is(sub, subLen, "scan")) {
        servo_scan();
    } else if (word_is(sub, subLen, "id")) {
        if (next_word(&args, &len, &w, &wl)) {
            if (!parse_num(w, wl, 253, &v)) {
                out("usage: servo id <0-253>\r\n");
                return;
            }
            servoId = (uint8_t)v;
        }
        out("talking to servo %u\r\n", servoId);
    } else if (word_is(sub, subLen, "ping")) {
        enum ServoResult r = servo_ping(servoId, 3000, &err);
        if (servo_report(r, err))
            out("servo %u answered\r\n", servoId);
    } else if (word_is(sub, subLen, "status")) {
        servo_status();
    } else if (word_is(sub, subLen, "angle") || word_is(sub, subLen, "pos")) {
        bool degrees = word_is(sub, subLen, "angle");
        uint64_t milli = 0;
        bool ok = next_word(&args, &len, &w, &wl);
        if (ok && degrees) {
            ok = parse_milli(w, wl, &milli) && milli <= 360000u;
            v = (uint32_t)((milli * 4096u + 180000u) / 360000u);
        } else if (ok) {
            ok = parse_num(w, wl, 4095, &v);
        }
        speed = servoSpeed;
        if (ok && next_word(&args, &len, &w, &wl))
            ok = parse_num(w, wl, 65535, &speed);
        if (ok && next_word(&args, &len, &w, &wl))
            ok = false;
        if (!ok) {
            out(degrees ? "usage: servo angle <0-360> [speed]\r\n" : "usage: servo pos <0-4095> [speed]\r\n");
            return;
        }
        servo_move(v, speed);
    } else if (word_is(sub, subLen, "speed") || word_is(sub, subLen, "acc")) {
        bool isSpeed = word_is(sub, subLen, "speed");
        if (!next_word(&args, &len, &w, &wl) || !parse_num(w, wl, isSpeed ? 65535u : 254u, &v)) {
            out(isSpeed ? "usage: servo speed <0-65535 steps/s> (0 = fastest)\r\n"
                        : "usage: servo acc <0-254> (0 = fastest)\r\n");
            return;
        }
        if (isSpeed)
            servoSpeed = v;
        else
            servoAcc = v;
        out("servo moves: speed %lu steps/s, acc %lu\r\n", (unsigned long)servoSpeed, (unsigned long)servoAcc);
    } else if (word_is(sub, subLen, "torque")) {
        uint8_t value;
        if (!next_word(&args, &len, &w, &wl) || !(word_is(w, wl, "on") || word_is(w, wl, "off"))) {
            out("usage: servo torque on | off\r\n");
            return;
        }
        if (word_is(w, wl, "on")) {
            servo_hold();               /* so it doesn't jump to an old goal */
            value = 1;
            if (servo_set(REG_TORQUE, &value, 1))
                out("servo torque on\r\n");
        } else {
            value = 0;
            if (servo_set(REG_TORQUE, &value, 1))
                out("servo torque off (free to turn by hand)\r\n");
        }
    } else if (word_is(sub, subLen, "stop")) {
        servo_hold();
    } else if (word_is(sub, subLen, "read")) {
        uint32_t addr, n = 1;
        if (!next_word(&args, &len, &w, &wl) || !parse_num(w, wl, 255, &addr) ||
            (next_word(&args, &len, &w, &wl) && !parse_num(w, wl, SERVO_MAX_DATA, &n)) || n == 0) {
            out("usage: servo read <addr> [len 1-%u]\r\n", SERVO_MAX_DATA);
            return;
        }
        uint8_t data[SERVO_MAX_DATA];
        if (!servo_get((uint8_t)addr, (uint8_t)n, data))
            return;
        for (uint32_t i = 0; i < n; i++) {
            if (i % 16u == 0)
                out(i ? "\r\n%3lu:" : "%3lu:", (unsigned long)(addr + i));
            out(" %02X", data[i]);
        }
        if (n == 1)
            out("   (%u)", data[0]);
        else if (n == 2)
            out("   (%u)", get16(data));
        out("\r\n");
    } else if (word_is(sub, subLen, "write")) {
        uint32_t addr;
        uint8_t data[SERVO_MAX_DATA];
        uint8_t n = 0;
        bool ok = next_word(&args, &len, &w, &wl) && parse_num(w, wl, 255, &addr);
        while (ok && next_word(&args, &len, &w, &wl)) {
            ok = n < SERVO_MAX_DATA - 1u && parse_num(w, wl, 255, &v);
            data[n++] = (uint8_t)v;
        }
        if (!ok || n == 0) {
            out("usage: servo write <addr> <byte> [byte...]   (EEPROM, below 40: write 55 0\r\n"
                "       first, write 55 1 after)\r\n");
            return;
        }
        if (servo_set((uint8_t)addr, data, n))
            out("wrote %u byte%s at %lu\r\n", n, n == 1 ? "" : "s", (unsigned long)addr);
    } else {
        out("unknown servo command \"%.*s\" (see help)\r\n", (int)subLen, sub);
    }
}

/* ---- Automatic mode ---- */

/* A time: ms (250, 250ms) or seconds (1.5s). */
static bool parse_ms(const char *s, size_t len, uint64_t *ms)
{
    if (len > 2 && lower(s[len - 2u]) == 'm' && lower(s[len - 1u]) == 's')
        return parse_u64(s, len - 2u, UINT32_MAX, ms);
    if (len > 1 && lower(s[len - 1u]) == 's')
        return parse_milli(s, len - 1u, ms);        /* thousandths of a second */
    return parse_u64(s, len, UINT32_MAX, ms);
}

static void put_duration(uint64_t ms)
{
    if (ms < 10000u) {
        uart_put_u64(ms);
        uart_puts(" ms");
        return;
    }
    uint64_t tenths = (ms + 50u) / 100u;
    if (tenths < 6000u) {
        uart_put_u64(tenths / 10u);
        uart_putc('.');
        uart_put_u64(tenths % 10u);
        uart_puts(" s");
        return;
    }
    uint64_t s = (ms + 500u) / 1000u;
    uart_put_u64(s / 60u);
    uart_puts(" min ");
    uart_put_u64(s % 60u);
    uart_puts(" s");
}

static void auto_print(void)
{
    uint32_t key = irq_lock();
    struct AutoSettings s = autoSet;
    irq_unlock(key);

    out("  pulse   %7lu ms   E on, each pulse\r\n", (unsigned long)s.pulseMs);
    out("  wait    %7lu ms   E off after each pulse\r\n", (unsigned long)s.waitMs);
    out("  timeout %7lu ms   reverse once this long has gone by one way (0: never)\r\n",
        (unsigned long)s.timeoutMs);
    out("  count   %7lu      or after this many pulses one way (0: no limit)\r\n", (unsigned long)s.count);
    out("  rest    %7lu ms   E off before each reversal\r\n", (unsigned long)s.restMs);
    out("  cycles  %7lu      there-and-back cycles, then stop (0: until stopped)\r\n",
        (unsigned long)s.cycles);
    out("  first   %7u      direction it starts in\r\n", s.firstDir ? 1u : 0u);

    uint32_t n = autorun_pulses_each_way(&s);
    if (n == 0) {
        uart_puts("  never reverses: it pulses one way until stopped (set a timeout or a count)\r\n");
        return;
    }
    uint64_t way = autorun_way_ms(&s);
    out("  each way: %lu pulse%s, ", (unsigned long)n, n == 1 ? "" : "s");
    put_duration(way);
    uart_puts("; repeats every ");
    put_duration(2u * way + 2u * (uint64_t)s.restMs);
    uart_puts(" (there, rest, back, rest)\r\n");
    if (s.cycles) {
        out("  %lu cycle%s: ", (unsigned long)s.cycles, s.cycles == 1u ? "" : "s");
        put_duration((uint64_t)s.cycles * 2u * way + (2u * (uint64_t)s.cycles - 1u) * s.restMs);
        uart_puts(", then it stops (no rest after the last pulse)\r\n");
    }
}

/* auto [start | stop | default | <name> <value>]...
 * Nothing changes unless the whole line is valid. */
static void cmd_auto(const char *args, size_t len)
{
    const char *w, *v;
    size_t wl, vl;
    bool start = false, stop = false, changed = false, firstChanged = false, any = false;

    uint32_t key = irq_lock();
    struct AutoSettings s = autoSet;
    irq_unlock(key);

    while (next_word(&args, &len, &w, &wl)) {
        any = true;
        if (word_is(w, wl, "start") || word_is(w, wl, "on") || word_is(w, wl, "go")) {
            start = true;
            stop = false;
            continue;
        }
        if (word_is(w, wl, "stop") || word_is(w, wl, "off")) {
            stop = true;
            start = false;
            continue;
        }
        if (word_is(w, wl, "default") || word_is(w, wl, "defaults")) {
            s = autoDefaults;
            changed = firstChanged = true;
            continue;
        }

        uint32_t *field = NULL;
        bool isTime = true, isDir = false;
        uint32_t min = 0, max = AUTO_MAX_MS;
        if (word_is(w, wl, "pulse")) {
            field = &s.pulseMs;
            min = 1;
            max = PULSE_MAX_MS;
        } else if (word_is(w, wl, "wait")) {
            field = &s.waitMs;
        } else if (word_is(w, wl, "timeout") || word_is(w, wl, "time")) {
            field = &s.timeoutMs;
        } else if (word_is(w, wl, "rest")) {
            field = &s.restMs;
            min = REVERSE_PAUSE_MS;
        } else if (word_is(w, wl, "count")) {
            field = &s.count;
            isTime = false;
            max = AUTO_MAX_COUNT;
        } else if (word_is(w, wl, "cycles") || word_is(w, wl, "cycle")) {
            field = &s.cycles;
            isTime = false;
            max = AUTO_MAX_COUNT;
        } else if (word_is(w, wl, "first")) {
            isDir = true;
        } else {
            out("auto: unknown setting \"%.*s\"; nothing changed (see help)\r\n", (int)wl, w);
            return;
        }

        if (!next_word(&args, &len, &v, &vl)) {
            out("auto %.*s: needs a value; nothing changed\r\n", (int)wl, w);
            return;
        }
        if (isDir) {
            if (word_is(v, vl, "0") || word_is(v, vl, "low"))
                s.firstDir = false;
            else if (word_is(v, vl, "1") || word_is(v, vl, "high"))
                s.firstDir = true;
            else {
                uart_puts("auto first: 0 or 1; nothing changed\r\n");
                return;
            }
            changed = firstChanged = true;
            continue;
        }
        uint64_t value;
        bool ok = isTime ? parse_ms(v, vl, &value) : parse_u64(v, vl, UINT32_MAX, &value);
        if (!ok || value < min || value > max) {
            out("auto %.*s: %lu to %lu%s; nothing changed\r\n", (int)wl, w, (unsigned long)min,
                (unsigned long)max, isTime ? " ms (or seconds, e.g. 1.5s)" : "");
            return;
        }
        *field = (uint32_t)value;
        changed = true;
    }

    if (!any) {
        auto_print();
        print_settings();
        return;
    }

    bool running = mode == MODE_AUTO;
    if (changed) {
        key = irq_lock();
        autoSet = s;
        irq_unlock(key);
        auto_print();
        if (running && !start && !stop) {
            uart_puts("(running: the new times apply from its next step");
            uart_puts(firstChanged ? ", first from the next start)\r\n" : ")\r\n");
        }
    }
    if (stop) {
        drive_set(false);
        uart_puts(running ? "automatic mode stopped\r\n" : "automatic mode wasn't running\r\n");
    }
    if (start) {
        if (running)
            uart_puts("automatic mode restarted from its first pulse\r\n");
        if (auto_start()) {
            uart_puts("E off for ");
            uart_put_u32(REVERSE_PAUSE_MS);
            uart_puts(" ms while the direction changes\r\n");
        }
    }
    print_settings();
}

static void run_line(const char *s, size_t len)
{
    const char *cmd;
    size_t cmdLen;

    if (!next_word(&s, &len, &cmd, &cmdLen))
        return;                         /* blank line */

    if (word_is(cmd, cmdLen, "dir"))
        cmd_dir(s, len);
    else if (word_is(cmd, cmdLen, "drive"))
        cmd_drive(s, len);
    else if (word_is(cmd, cmdLen, "pulse"))
        cmd_pulse(s, len);
    else if (word_is(cmd, cmdLen, "auto"))
        cmd_auto(s, len);
    else if (word_is(cmd, cmdLen, "stop")) {
        bool wasAuto = mode == MODE_AUTO;
        drive_set(false);
        if (wasAuto)
            uart_puts("automatic mode stopped\r\n");
        print_settings();
    } else if (word_is(cmd, cmdLen, "pwm"))
        cmd_pwm(s, len);
    else if (word_is(cmd, cmdLen, "servo"))
        cmd_servo(s, len);
    else if (word_is(cmd, cmdLen, "status"))
        print_status();
    else if (word_is(cmd, cmdLen, "help") || word_is(cmd, cmdLen, "?"))
        print_help();
    else {
        uart_puts("unknown command \"");
        uart_write(cmd, cmdLen);
        uart_puts("\", type help\r\n");
    }
}

/* ---- Input bytes ---- */

enum EscState { ESC_NONE, ESC_START, ESC_SEQUENCE };

static void handle_byte(uint8_t c)
{
    static uint8_t prev;
    static enum EscState esc = ESC_NONE;
    static bool escParams;

    /* Escape sequences: Up / Down for history, the rest ignored. */
    if (esc == ESC_START) {
        esc = (c == '[' || c == 'O') ? ESC_SEQUENCE : ESC_NONE;
        if (esc == ESC_SEQUENCE) {
            escParams = false;
            return;
        }
    } else if (esc == ESC_SEQUENCE) {
        if (c >= 0x20 && c <= 0x3F) {
            escParams = true;           /* e.g. Ctrl-Up: Esc [ 1 ; 5 A */
            return;
        }
        esc = ESC_NONE;
        if (c >= 0x40 && c <= 0x7E) {   /* final byte */
            if (!escParams && c == 'A')
                history_up();
            else if (!escParams && c == 'B')
                history_down();
            return;
        }
    }

    if (c == 0x1B) {
        esc = ESC_START;
    } else if (c == 0x03) {             /* Ctrl-C: drive already off; drop the line */
        uart_puts("^C\r\n");
        lineLen = 0;
        lineOverflow = false;
        browse = -1;
        prompt();
    } else if (c == '\r' || c == '\n') {
        if (!(c == '\n' && prev == '\r')) {     /* CR LF counts once */
            uart_puts("\r\n");
            if (lineOverflow) {
                uart_puts("line too long (max ");
                uart_put_u32(LINE_MAX);
                uart_puts(" characters), ignored\r\n");
            } else {
                history_add(line, lineLen);
                run_line(line, lineLen);
            }
            lineLen = 0;
            lineOverflow = false;
            browse = -1;
            prompt();
        }
    } else if (c == 0x08 || c == 0x7F) {        /* Backspace / Delete */
        if (lineLen > 0) {
            lineLen--;
            uart_puts("\b \b");
        }
    } else if (c >= 0x20 && c < 0x7F) {
        if (lineLen < LINE_MAX) {
            line[lineLen++] = (char)c;
            uart_putc((char)c);                 /* echo: picocom doesn't show what you type */
        } else {
            lineOverflow = true;
        }
    }
    /* other control bytes: ignored */
    prev = c;
}

/* ---- Events, LEDs, limits ---- */

static void handle_events(void)
{
    uint32_t key = irq_lock();
    uint32_t ev = events;
    events = 0;
    irq_unlock(key);

    if (ev == 0)
        return;

    async_begin();
    if (ev & EV_PULSE_DONE)
        uart_puts("pulse done\r\n");
    if (ev & EV_STOP_KEY)
        uart_puts("STOP (Esc / Ctrl-C): drive and PWM off\r\n");
    if (ev & EV_STOP_SERIAL)
        uart_puts("STOP: serial line error, drive and PWM off (unplugged? wrong baud rate?)\r\n");
    if (ev & (EV_AUTO_REVERSE | EV_AUTO_DONE)) {
        key = irq_lock();
        struct AutoRun r = autoRun;
        uint32_t cycles = autoSet.cycles;
        bool high = dirHigh;
        irq_unlock(key);
        if (ev & EV_AUTO_REVERSE) {
            out("auto: reversed, dir %u now, cycle %lu", high ? 1u : 0u, (unsigned long)(r.ways / 2u + 1u));
            if (cycles)
                out(" of %lu", (unsigned long)cycles);
            out(", %lu pulse%s so far\r\n", (unsigned long)r.total, r.total == 1u ? "" : "s");
        }
        if (ev & EV_AUTO_DONE)
            out("auto done: %lu cycle%s, %lu pulses\r\n", (unsigned long)(r.ways / 2u),
                r.ways / 2u == 1u ? "" : "s", (unsigned long)r.total);
    }
    if ((ev & EV_STOP_KEY) && servoUsed)
        servo_hold();
    print_settings();
    async_end();
}

static void led_set(enum Led i, bool on)
{
    bool high = LED_ACTIVE_HIGH ? on : !on;
    pin_write(leds[i].j5, high);
    pin_write(leds[i].board, high);
}

static void leds_update(uint32_t now)
{
    bool want[LED_COUNT];
    want[LED_HEARTBEAT] = now % HEARTBEAT_MS < HEARTBEAT_MS / 2u;
    want[LED_DRIVE] = motor_running();
    want[LED_DIR] = pin_driven(PIN_M);
    want[LED_SERIAL] = (rxSeen && now - lastRxMs < SERIAL_FLASH_MS) ||
                       (errorSeen && now - lastErrorMs < SERIAL_ERROR_MS);

    static bool shown[LED_COUNT];       /* all off after the lamp test */
    for (int i = 0; i < LED_COUNT; i++) {
        if (want[i] != shown[i]) {
            led_set((enum Led)i, want[i]);
            shown[i] = want[i];
        }
    }
}

static void lamp_test(void)
{
    for (int i = 0; i < LED_COUNT; i++) {
        led_set((enum Led)i, true);
        delay_ms(LAMP_TEST_MS);
        led_set((enum Led)i, false);
    }
}

struct Limit {
    const struct Pin *pin;
    const char *name;
    bool stable;                        /* debounced: pressed */
    uint32_t changingFor;               /* ms the raw reading has differed from stable */
};

static struct Limit limits[2] = {
    {&PIN_LIMIT_MAX, "EXT_MAX (PC2)", false, 0},
    {&PIN_LIMIT_MIN, "EXT_MIN (PC3)", false, 0},
};

static void limits_init(void)
{
    for (int i = 0; i < 2; i++)
        limits[i].stable = limit_pressed(*limits[i].pin);
}

static void limits_poll(void)            /* once per ms */
{
    for (int i = 0; i < 2; i++) {
        struct Limit *l = &limits[i];
        if (limit_pressed(*l->pin) == l->stable) {
            l->changingFor = 0;
        } else if (++l->changingFor >= LIMIT_DEBOUNCE_MS) {
            l->stable = !l->stable;
            l->changingFor = 0;
            async_begin();
            uart_puts("limit ");
            uart_puts(l->name);
            uart_puts(l->stable ? " pressed\r\n" : " released\r\n");
            async_end();
        }
    }
}

/* Heap for newlib's vsnprintf: from the end of .bss up to the stack. */
void *_sbrk(ptrdiff_t incr);

void *_sbrk(ptrdiff_t incr)
{
    extern uint8_t _end, _estack, _Min_Stack_Size;
    static uint8_t *heapEnd = &_end;
    uint8_t *limit = &_estack - (uint32_t)&_Min_Stack_Size;
    if (heapEnd + incr > limit)
        return (void *)-1;
    uint8_t *prev = heapEnd;
    heapEnd += incr;
    return prev;
}

/* ---- Main ---- */

int main(void)
{
    motor_pins_init();                  /* motor off before anything else */
    clock_init();
    SystemCoreClockUpdate();            /* 84 MHz (16 if the PLL didn't start) */
    read_reset_cause();
    other_pins_init();
    pwm_init();

    NVIC_SetPriorityGrouping(0);
    SysTick_Config(SystemCoreClock / 1000u);    /* 1 ms tick */
    NVIC_SetPriority(SysTick_IRQn, IRQ_PRIORITY);
    console_init();
    servo_port_init();
    watchdog_start();

    uart_puts("\r\n\r\n=== PCB motor test (STM32F401) ===\r\n"
              "Console: J7 pins 5/6, 115200 8N1 (software UART). Servo: J7 pin 3, 1 Mbaud.\r\n"
              "Motor: E = PA0 (J1 pin 5), M = PE0 (J1 pin 4).\r\n"
              "Reset cause: ");
    uart_puts(resetCause);
    if (SystemCoreClock != 84000000u)
        uart_puts("\r\n!! running at 16 MHz: the PLL didn't start");
    uart_puts("\r\n\r\n");
    print_help();
    uart_puts("\r\n");
    print_settings();
    prompt();

    lamp_test();                        /* each LED in turn: heartbeat, drive, direction, serial */
    limits_init();

    uint32_t lastTick = msTicks;
    for (;;) {
        IWDG->KR = 0xAAAA;              /* feed the watchdog */

        uint32_t now = msTicks;
        leds_update(now);
        while (lastTick != now) {       /* once per ms, catching up if we fell behind */
            lastTick++;
            limits_poll();
        }

        handle_events();

        uint8_t c;
        if (uart_read(&c))
            handle_byte(c);
    }
}

/**
 * @file    main.c
 * @brief   Motor test for the PCB (STM32F401VCT6).
 *
 * A serial command prompt for the arm motor on J1 (DRI0002: E = enable,
 * M = direction), plus status LEDs on J5.
 *
 *   dir 0 | 1 | toggle     direction pin M (PE0) low or high
 *   drive on | off         E (PA6) on at 100 % (held high), or off
 *   pulse <ms>             E on for <ms> milliseconds (1 to 60000), then off
 *   stop                   drive off
 *   status                 pins, limit switches, uptime, reset cause, serial errors
 *   help
 *   Esc or Ctrl-C          drive off at once (doesn't need Enter)
 *   Up / Down arrows       step through the last HISTORY_SIZE commands
 *   pwm ...                PWM test on the header pins a timer can drive (see help)
 *
 * The settings are printed after every command, and again whenever
 * something changes on its own (a pulse ends, a limit switch changes).
 *
 * Serial: USART1 on J7, PA9 TX / PA10 RX, 115200 8N1. Same as the
 * usb-uart-bridge's default, so `picocom -b 115200 /dev/ttyACM0` works.
 * Received bytes go into a ring buffer from the interrupt. Sent bytes go
 * into another ring buffer and out on the TX-empty interrupt, so printing
 * never holds up the motor timing.
 *
 * Reliability:
 *   - The motor pins are set to "off" before they become outputs, so
 *     there's no twitch at reset.
 *   - The pulse is timed in the SysTick interrupt: it starts on a tick and
 *     ends exactly <ms> ticks later, whatever the main loop is doing.
 *   - Esc and Ctrl-C are acted on in the interrupts, so they stop the
 *     drive even if the main loop is busy. Ctrl-C stops at once; Esc stops
 *     ESC_WAIT_MS later unless it turns out to be the start of an arrow key.
 *   - Any serial line error (framing, noise, overrun, receive buffer full)
 *     while the drive is on stops it. Unplugging the Blackpill shows up as
 *     a framing error, so losing the link stops the motor.
 *   - Changing direction while the motor runs holds E off for
 *     REVERSE_PAUSE_MS first, so the driver never reverses at full power.
 *   - Independent watchdog: if the main loop stops for about 1 s, the chip
 *     resets (motor off). The reset cause is printed at start-up.
 *   - A HardFault turns the motor off, says so on the serial port, and
 *     resets.
 *
 * Clock: the reset default, HSI 16 MHz, no PLL (the PCB has no crystal).
 * HSI is within 1 % at room temperature, so pulse lengths are too.
 *
 * Register level, no HAL: CMSIS headers, the start-up file and
 * system_stm32f4xx.c only.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "stm32f4xx.h"

/* ================================================================ settings */

#define BAUD                115200u
#define PULSE_MAX_MS        60000u
#define REVERSE_PAUSE_MS    100u    /* E held off this long when the direction changes while running */
#define DIR_AT_RESET        0       /* M level at reset: 0 = low, 1 = high */
#define E_ON_LEVEL          1       /* E level that runs the motor (DRI0002: high = on) */

#define LIMIT_ACTIVE_HIGH   1       /* switches to 3.3 V with pull-downs: pressed = high */
#define LIMIT_DEBOUNCE_MS   20u

#define LED_ACTIVE_HIGH     1       /* LED from the pin through a resistor to GND */
#define HEARTBEAT_MS        1000u   /* heartbeat period: on for half, off for half */
#define LAMP_TEST_MS        250u    /* each LED lit this long, in turn, at start-up */
#define SERIAL_FLASH_MS     30u     /* serial LED on this long per byte received */
#define SERIAL_ERROR_MS     2000u   /* ...and this long after a line error */

#define WATCHDOG_MS         1000u   /* nominal: the LSI is 17-47 kHz, so 0.7 to 1.9 s */

#define LINE_MAX            80u     /* longest command line */
#define HISTORY_SIZE        16u     /* commands kept for the Up arrow */
#define ESC_WAIT_MS         20u     /* an Esc with nothing after it for this long is the Esc key */
#define RX_SIZE             1024u   /* receive ring, power of two */
#define TX_SIZE             4096u   /* transmit ring, power of two */

#define PWM_DEFAULT_MHZ     1000000u    /* PWM default frequency, thousandths of a Hz: 1 kHz */
#define PWM_DEFAULT_DUTY    50000u      /* PWM default duty, thousandths of a %: 50 % */
#define PWM_MEASURE_MS      200u        /* how long 'pwm' samples the pins for */

#define IRQ_PRIORITY        1u      /* SysTick and USART1: same level, so they never interrupt each other */

_Static_assert((RX_SIZE & (RX_SIZE - 1)) == 0, "RX_SIZE must be a power of two");
_Static_assert((TX_SIZE & (TX_SIZE - 1)) == 0, "TX_SIZE must be a power of two");
_Static_assert(WATCHDOG_MS >= 100 && WATCHDOG_MS <= 4096, "WATCHDOG_MS: 100 to 4096");

/* ================================================================ pins */

struct Pin {
    GPIO_TypeDef *port;
    uint32_t n;                         /* 0 to 15 */
};

/* Motor: J1 (LEFT motor header) */
static const struct Pin PIN_E = {GPIOA, 6};         /* J1 pin 3, LEFT_PWM: DRI0002 E */
static const struct Pin PIN_M = {GPIOE, 0};         /* J1 pin 4, LEFT_DIR: DRI0002 M */

/* Arm limit switches */
static const struct Pin PIN_LIMIT_MAX = {GPIOC, 2}; /* EXT_MAX */
static const struct Pin PIN_LIMIT_MIN = {GPIOC, 3}; /* EXT_MIN */

/* Serial: J7 */
static const struct Pin PIN_TX = {GPIOA, 9};        /* USART1_TX, AF7 */
static const struct Pin PIN_RX = {GPIOA, 10};       /* USART1_RX, AF7 */

/* LEDs. Each one is shown on a J5 pin, and also on the board LED pins
 * PD0-PD3 (which go to J4, not fitted). */
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
    p.port->BSRR = high ? (1u << p.n) : (1u << (p.n + 16u));   /* atomic: safe from anywhere */
}

static inline bool pin_read(struct Pin p)           /* the level on the pin */
{
    return (p.port->IDR >> p.n) & 1u;
}

static inline bool pin_driven(struct Pin p)         /* the level we're driving it to */
{
    return (p.port->ODR >> p.n) & 1u;
}

/* Push-pull output, low speed. The level is set first, so the pin goes
 * straight to it with no glitch. Only called before interrupts start. */
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

/* ================================================================ time */

static volatile uint32_t msTicks;

static void delay_ms(uint32_t ms)       /* start-up only: feeds the watchdog */
{
    uint32_t start = msTicks;
    while (msTicks - start < ms)
        IWDG->KR = 0xAAAA;
}

/* ================================================================ PWM test outputs
 *
 * The header pins a timer can drive, for finding out which ones give PWM.
 * Pins on the same timer share its frequency; each pin has its own duty
 * and polarity. A pin that's off is left as an input, as it is at reset,
 * except J1 pin 3: that's the motor's E, so it goes back to the drive.
 * The timers run all the time; turning a pin on just hands it to its timer. */

enum { TIM_2, TIM_3, TIM_4, TIMER_COUNT };

static const struct {
    const char *name;
    TIM_TypeDef *tim;
    bool is32;                          /* 32-bit counter (TIM2), else 16-bit */
} pwmTimers[TIMER_COUNT] = {
    [TIM_2] = {"TIM2", TIM2, true},
    [TIM_3] = {"TIM3", TIM3, false},
    [TIM_4] = {"TIM4", TIM4, false},
};

#define PWM_E   0                       /* J1 pin 3 = PA6 = the motor's E */

static const struct {
    const char *name;                   /* header pin */
    const char *chip;
    const char *net;
    struct Pin pin;
    uint32_t af;
    uint8_t timer;
    uint8_t ch;                         /* timer channel, 1 to 4 */
    bool inAll;                         /* switched by 'pwm on' and 'all' */
} pwmChannels[] = {
    [PWM_E] = {"j1.3", "PA6", "LEFT_PWM",  {GPIOA, 6}, 2, TIM_3, 1, false},
              {"j1.5", "PA0", "LEFT_FG",   {GPIOA, 0}, 1, TIM_2, 1, true},
              {"j1.6", "PB0", "DRILL_PWM", {GPIOB, 0}, 2, TIM_3, 3, true},
              {"j2.3", "PA7", "RIGHT_PWM", {GPIOA, 7}, 2, TIM_3, 2, true},
              {"j2.5", "PA1", "RIGHT_FG",  {GPIOA, 1}, 1, TIM_2, 2, true},
              {"j2.8", "PA2", "SERVO_SIG", {GPIOA, 2}, 1, TIM_2, 3, true},
              {"j7.3", "PB6", "I2C1_SCL",  {GPIOB, 6}, 2, TIM_4, 1, true},
              {"j7.4", "PB7", "I2C1_SDA",  {GPIOB, 7}, 2, TIM_4, 2, true},
};
#define PWM_COUNT ((unsigned)(sizeof pwmChannels / sizeof pwmChannels[0]))

static volatile bool pwmOn[PWM_COUNT];
static uint32_t pwmDuty[PWM_COUNT];     /* thousandths of a %: 0 to 100000 */
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
        pin_output(pwmChannels[i].pin, !E_ON_LEVEL);    /* back to the drive, which is off */
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

/* ================================================================ drive
 *
 * All drive state is changed either in the SysTick interrupt or in main
 * with interrupts off, and E is always set from that state by
 * drive_apply(), so the pin can't disagree with it. */

enum DriveMode { MODE_OFF, MODE_ON, MODE_PULSE };

/* Things the interrupts tell the main loop about. */
#define EV_PULSE_DONE   (1u << 0)
#define EV_STOP_KEY     (1u << 1)       /* Esc or Ctrl-C stopped the drive */
#define EV_STOP_SERIAL  (1u << 2)       /* a serial line error stopped the drive */

static volatile enum DriveMode mode = MODE_OFF;
static volatile uint32_t pulseMs;       /* length of the current pulse */
static volatile uint32_t pulseLeftMs;   /* E-on time still to go */
static volatile bool pulseWaitTick;     /* pulse starts on the next tick */
static volatile uint32_t pauseLeftMs;   /* E held off after a direction change */
static volatile uint32_t stoppedForMs = REVERSE_PAUSE_MS;  /* E off this long, up to REVERSE_PAUSE_MS */
static volatile bool dirHigh = DIR_AT_RESET;
static volatile uint32_t events;
static volatile bool escPending;        /* Esc received; waiting to see if it starts an arrow key */
static volatile uint32_t escAtMs;

static bool motor_should_run(void)
{
    return mode != MODE_OFF && pauseLeftMs == 0 && !pulseWaitTick;
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

/* Drive and every PWM pin off. From an interrupt (same priority as
 * SysTick, so not interrupted by it). */
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

/* E on for ms milliseconds, then off. From stopped, E goes on at the next
 * tick and off exactly ms ticks later. If E is already on, it stays on (no
 * gap) and goes off ms later, to within 1 ms. */
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

/* If the motor is running, or was in the last REVERSE_PAUSE_MS (it may
 * still be turning), E is held off for REVERSE_PAUSE_MS from now. Returns
 * true if the drive is on and so is being paused. */
static bool drive_set_dir(bool high)
{
    bool paused = false;
    uint32_t key = irq_lock();
    if (high != dirHigh) {
        if (motor_should_run() || pauseLeftMs > 0 || stoppedForMs < REVERSE_PAUSE_MS) {
            pauseLeftMs = REVERSE_PAUSE_MS;
            drive_apply();              /* E off before M changes */
            paused = (mode != MODE_OFF);
        }
        dirHigh = high;
        pin_write(PIN_M, high);
    }
    irq_unlock(key);
    return paused;
}

void SysTick_Handler(void)
{
    msTicks++;

    if (escPending && msTicks - escAtMs >= ESC_WAIT_MS) {
        escPending = false;             /* nothing followed: it was the Esc key */
        stop_all_isr(EV_STOP_KEY);
    }

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
    }
    drive_apply();

    if (motor_should_run())
        stoppedForMs = 0;
    else if (stoppedForMs < REVERSE_PAUSE_MS)
        stoppedForMs++;
}

/* ================================================================ serial */

static volatile uint8_t rxBuf[RX_SIZE];
static volatile uint32_t rxHead, rxTail;    /* free-running; head: interrupt, tail: main */
static volatile uint8_t txBuf[TX_SIZE];
static volatile uint32_t txHead, txTail;    /* head: main, tail: interrupt */

static volatile struct {
    uint32_t framing, noise, overrun, lost;
} serialErrors;

static volatile uint32_t lastRxMs, lastErrorMs;
static volatile bool rxSeen, errorSeen;

void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;

    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        /* Reading SR then DR clears RXNE and all the error flags. */
        uint8_t byte = (uint8_t)USART1->DR;
        bool problem = false;

        if (sr & USART_SR_ORE) {        /* a byte was lost; this one is good */
            serialErrors.overrun++;
            problem = true;
        }
        if (sr & (USART_SR_FE | USART_SR_PE)) {
            serialErrors.framing++;     /* wrong baud rate, or the line dropped */
            problem = true;
        }
        if (sr & USART_SR_NE) {
            serialErrors.noise++;
            problem = true;
        }

        if ((sr & USART_SR_RXNE) && !(sr & (USART_SR_FE | USART_SR_NE | USART_SR_PE))) {
            lastRxMs = msTicks;
            rxSeen = true;
            /* Ctrl-C stops at once. An arrow key arrives as Esc [ A (or
             * Esc O A) all together, so Esc followed straight away by [ or O
             * isn't a stop; any other byte after it, or nothing within
             * ESC_WAIT_MS (see SysTick), is. */
            if (escPending) {
                escPending = false;
                if (byte != '[' && byte != 'O')
                    stop_all_isr(EV_STOP_KEY);
            }
            if (byte == 0x1B) {
                escAtMs = msTicks;
                escPending = true;
            } else if (byte == 0x03) {
                stop_all_isr(EV_STOP_KEY);
            }
            if (rxHead - rxTail < RX_SIZE) {
                rxBuf[rxHead % RX_SIZE] = byte;
                rxHead++;
            } else {
                serialErrors.lost++;
                problem = true;
            }
        }

        if (problem) {
            lastErrorMs = msTicks;
            errorSeen = true;
            stop_all_isr(EV_STOP_SERIAL);
        }
    }

    if ((USART1->CR1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
        if (txTail != txHead) {
            USART1->DR = txBuf[txTail % TX_SIZE];
            txTail++;
        } else {
            USART1->CR1 &= ~USART_CR1_TXEIE;        /* nothing left to send */
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
    /* If the ring is full, wait for the interrupt to send some. That takes
     * at most one byte time (87 us); if sending ever stopped altogether, the
     * watchdog would reset the chip. */
    while (txHead - txTail >= TX_SIZE) {
    }
    txBuf[txHead % TX_SIZE] = (uint8_t)c;
    txHead++;
    USART1->CR1 |= USART_CR1_TXEIE;
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

/* ================================================================ set-up */

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

static void uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;

    /* 16x oversampling: BRR = PCLK2 / baud, rounded. 16 MHz / 115200 = 139,
     * 0.08 % slow. */
    static const uint8_t apbShift[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    uint32_t pclk2 = SystemCoreClock >> apbShift[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos];
    USART1->CR1 = 0;
    USART1->CR2 = 0;                    /* 1 stop bit */
    USART1->CR3 = 0;
    USART1->BRR = (pclk2 + BAUD / 2u) / BAUD;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;   /* 8N1 */

    /* Pins after the USART is on, so TX goes straight to idle (high) with
     * no false start bit. Pull-up on RX: with nothing plugged in it reads
     * as an idle line, not noise. */
    pin_alternate(PIN_TX, 7, 0);
    pin_alternate(PIN_RX, 7, 1);

    NVIC_SetPriority(USART1_IRQn, IRQ_PRIORITY);
    NVIC_EnableIRQ(USART1_IRQn);
}

static void watchdog_start(void)
{
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;     /* paused while a debugger has the core halted */
    IWDG->KR = 0xCCCC;                  /* start (this also starts the LSI) */
    IWDG->KR = 0x5555;                  /* unlock PR and RLR */
    IWDG->PR = 3;                       /* /32: about 1 kHz from the 32 kHz LSI */
    IWDG->RLR = WATCHDOG_MS - 1u;
    for (uint32_t n = 0; n < 1000000u && IWDG->SR != 0; n++) {
    }                                   /* wait for the new values to load */
    IWDG->KR = 0xAAAA;
}

/* ================================================================ PWM timers */

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

/* Raw prescaler and auto-reload. The pins' compare values follow, to keep
 * their duty, and the counter restarts so the change shows at once. */
static void pwm_timer_set(unsigned ti, uint32_t psc, uint32_t arr)
{
    TIM_TypeDef *t = pwmTimers[ti].tim;
    t->PSC = psc;
    t->ARR = arr;                       /* preloaded: takes effect with the UG below */
    t->EGR = TIM_EGR_UG;
    for (unsigned i = 0; i < PWM_COUNT; i++)
        if (pwmChannels[i].timer == ti)
            pwm_apply_duty(i);
}

/* Sets a timer as close as it can get to milliHz (thousandths of a Hz),
 * with the finest duty steps possible. False if out of range. */
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

/* All pins off, every timer at 1 kHz, every pin 50 % and normal polarity. */
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

    /* Timers on APB1 count at PCLK1, or twice that if APB1 is divided. */
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
    /* Each pin's channel: PWM mode 1 (high while counter < compare), no
     * compare preload so a duty change shows at once, output enabled. The
     * pins themselves stay as they are until they're turned on. */
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

/* ================================================================ faults */

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
    USART1->CR1 &= ~USART_CR1_TXEIE;

    static const char head[] = "\r\n\r\n*** ";
    static const char tail[] = ": motor and PWM off, resetting ***\r\n";
    const char *parts[3] = {head, what, tail};
    for (int p = 0; p < 3; p++) {
        for (const char *s = parts[p]; *s; s++) {
            for (uint32_t n = 0; n < 100000u && !(USART1->SR & USART_SR_TXE); n++) {
            }
            USART1->DR = (uint8_t)*s;
        }
    }
    for (uint32_t n = 0; n < 100000u && !(USART1->SR & USART_SR_TC); n++) {
    }
    NVIC_SystemReset();
}

void HardFault_Handler(void)  { fault_stop("HardFault"); }
void NMI_Handler(void)        { fault_stop("NMI"); }
void MemManage_Handler(void)  { fault_stop("MemManage fault"); }
void BusFault_Handler(void)   { fault_stop("BusFault"); }
void UsageFault_Handler(void) { fault_stop("UsageFault"); }

/* ================================================================ display */

static void put_level(bool high)
{
    uart_puts(high ? "high" : "low");
}

/* Warns if a pin we're driving reads back at the other level (shorted to
 * GND or 3.3 V, or overloaded). */
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
    irq_unlock(key);

    if (m == MODE_OFF) {
        uart_puts("off");
    } else if (m == MODE_ON) {
        uart_puts("on");
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
        put_pin_check(PIN_E, "E (PA6)");
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
    uart_puts("   E = PA6, J1 pin 3, ");
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
    uart_puts(" noise, ");
    uart_put_u32(serialErrors.overrun);
    uart_puts(" overrun errors, ");
    uart_put_u32(serialErrors.lost);
    uart_puts(" bytes lost (buffer full)\r\n");

    bool bad = false;
    if ((!pwmOn[PWM_E] && pin_read(PIN_E) != pin_driven(PIN_E)) || pin_read(PIN_M) != pin_driven(PIN_M))
        bad = true;
    if (bad) {
        uart_puts("pins      ");
        if (!pwmOn[PWM_E])
            put_pin_check(PIN_E, "E (PA6)");
        put_pin_check(PIN_M, "M (PE0)");
        uart_puts("\r\n");
    }
}

static void print_help(void)
{
    uart_puts("Commands:\r\n"
              "  dir 0 | 1 | toggle   direction pin M (PE0, J1 pin 4) low or high\r\n"
              "  drive on | off       E (PA6, J1 pin 3) on at 100 %, or off\r\n"
              "  pulse <ms>           E on for <ms> milliseconds (1 to 60000), then off\r\n"
              "  stop                 drive off\r\n"
              "  status               pins, limit switches, uptime, reset cause, serial errors\r\n"
              "  help\r\n"
              "PWM test (<pin>: j1.3 j1.5 j1.6 j2.3 j2.5 j2.8 j7.3 j7.4, or pa6 etc.):\r\n"
              "  pwm                  list pins and timers, and measure each pin\r\n"
              "  pwm on | off         all pins on / off (not j1.3, the motor's E)\r\n"
              "  pwm default          all off, 1 kHz, 50 %, normal polarity\r\n"
              "  pwm <pin> on | off\r\n"
              "  pwm <pin> duty <%>   e.g. 37.5         (or all / tim2 / tim3 / tim4)\r\n"
              "  pwm <pin> pol normal | inverted        (or all / tim2 / tim3 / tim4)\r\n"
              "  pwm <pin> ccr <n>    raw compare; n = arr + 1 is 100 %\r\n"
              "  pwm <tim> freq <Hz>  e.g. 20k, 0.5, 2M (or a pin: sets its timer; or all)\r\n"
              "  pwm <tim> psc <n>    raw prescaler     (or a pin: sets its timer)\r\n"
              "  pwm <tim> arr <n>    raw auto-reload   (or a pin: sets its timer)\r\n"
              "Esc or Ctrl-C turns the drive and all PWM off at once, without Enter.\r\n"
              "Up / Down arrows: previous commands.\r\n"
              "LEDs on J5: pin 3 heartbeat, 4 drive, 5 direction, 6 serial.\r\n");
}

/* ================================================================ line input */

static char line[LINE_MAX];
static size_t lineLen;
static bool lineOverflow;

static void prompt(void)
{
    uart_puts("> ");
}

/* For messages that aren't a reply to a command: they go on their own
 * line, then the prompt and whatever was being typed come back. */
static void async_begin(void)
{
    uart_puts("\r\n");
}

static void async_end(void)
{
    prompt();
    uart_write(line, lineLen);
}

/* ---- history: Up and Down step through earlier commands ---- */

static char history[HISTORY_SIZE][LINE_MAX];
static size_t historyLen[HISTORY_SIZE];
static uint32_t historyTotal;           /* commands ever stored; the newest is at (historyTotal - 1) % HISTORY_SIZE */
static int browse = -1;                 /* -1: typing a new line; 0: newest command shown; 1: the one before... */
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

/* Stores a command that was run. Blank lines and repeats of the newest
 * aren't stored. */
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

/* Takes the next space-separated word. Returns false if there isn't one. */
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

/* ================================================================ commands */

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
    if (word_is(w, wl, "on"))
        drive_set(true);
    else if (word_is(w, wl, "off"))
        drive_set(false);
    else {
        uart_puts("usage: drive on | off\r\n");
        return;
    }
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
    drive_pulse(ms);
    print_settings();
}

/* ---- PWM test commands ---- */

/* Text built up in a buffer, for lining up columns. */
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

/* v in thousandths, printed with up to 3 decimals, trailing zeros dropped. */
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

/* Case-insensitive whole-word compare against a name in any case. */
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

/* A number with up to 3 decimals and an optional k or M (x1000, x1000000)
 * or % suffix, in thousandths: "1.5k" -> 1500000. */
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

/* How much of the time each pin that's on reads high, sampled over
 * PWM_MEASURE_MS. This reads the pin itself, so a pin that's dead or held
 * down shows 0 % whatever the timer is doing. Returns the sample count. */
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
        uart_puts("j1.3 is the motor's E: drive off, the PWM test drives it now\r\n");
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

    /* Target: which pins (and timers) the action applies to. "all" and a
     * timer cover every pin they hold, except that they don't turn the
     * motor's E (j1.3) on or off: only naming it does. */
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
    else if (word_is(cmd, cmdLen, "stop")) {
        drive_set(false);
        print_settings();
    } else if (word_is(cmd, cmdLen, "pwm"))
        cmd_pwm(s, len);
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

/* ================================================================ input bytes */

enum EscState { ESC_NONE, ESC_START, ESC_SEQUENCE };

static void handle_byte(uint8_t c)
{
    static uint8_t prev;
    static enum EscState esc = ESC_NONE;
    static bool escParams;

    /* The interrupts deal with Esc as a stop key. Here escape sequences are
     * read: Up and Down step through the history, and the rest are
     * swallowed so they don't type "[C" etc. into the line. */
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

/* ================================================================ events, LEDs, limits */

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

/* ================================================================ main */

int main(void)
{
    motor_pins_init();                  /* motor off before anything else */
    SystemCoreClockUpdate();            /* 16 MHz (HSI) */
    read_reset_cause();
    other_pins_init();
    pwm_init();

    NVIC_SetPriorityGrouping(0);
    SysTick_Config(SystemCoreClock / 1000u);    /* 1 ms tick */
    NVIC_SetPriority(SysTick_IRQn, IRQ_PRIORITY);
    uart_init();
    watchdog_start();

    uart_puts("\r\n\r\n=== PCB motor test (STM32F401) ===\r\n"
              "Serial: USART1 on J7, 115200 8N1. Motor: E = PA6 (J1 pin 3), M = PE0 (J1 pin 4).\r\n"
              "Reset cause: ");
    uart_puts(resetCause);
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

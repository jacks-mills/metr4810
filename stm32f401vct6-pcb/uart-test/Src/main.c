/**
 * @file    main.c
 * @brief   UART terminal test for the PCB (STM32F401VCT6).
 *
 * Checks the USB-serial adapter on J7 and the four LEDs before running
 * robot. Same port and settings as the robot terminal: USART1,
 * PA9 = TX, PA10 = RX, 115200 8N1.
 *
 * Commands (any case), one per line, ended by Enter (CR, LF or CR LF):
 *   led on | led off | led toggle | led status        all four LEDs
 *   led <colour> on | off | toggle | status           one LED
 *       colours: red (PD0), orange (PD1), green (PD2), start (PD3)
 *   echo <text>        sends back everything after "echo "
 *   help
 *
 * Characters are echoed as you type them (terminal programs like picocom
 * don't show what you type), with Backspace working. Control bytes show as
 * <hex>, e.g. an arrow key as <1B>[A.
 *
 * LEDs: assumed lit when the pin is high, like robot's status_led_pcb.c.
 * If "led on" turns them off, set LED_ACTIVE_HIGH to 0 (and change LED_ON
 * in robot to match).
 *
 * Clock: the PCB has no crystal; left at the reset default, HSI 16 MHz with
 * no PLL. USART1 runs from APB2 = 16 MHz.
 *
 * Register level, no HAL: just CMSIS headers, the start-up file and
 * system_stm32f4xx.c. Bytes come in on the RXNE interrupt into a ring
 * buffer; sending is polled.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx.h"

#define BAUD            115200u
#define LINE_MAX        80      /* longest line kept */
#define RX_RING_SIZE    256     /* power of two */
#define LED_ACTIVE_HIGH 1       /* 1: pin high = lit, 0: pin low = lit */

/* ---------------------------------------------------------------- state */

static volatile uint8_t  rxRing[RX_RING_SIZE];
static volatile uint32_t rxHead;            /* written by the interrupt */
static uint32_t          rxTail;            /* written by main */

/* ---------------------------------------------------------------- LEDs */

#define LED_COUNT 4

static const struct {
    const char *name;
    const char *pinName;
    uint32_t pin;                       /* PD0..PD3 */
} leds[LED_COUNT] = {
    {"red",    "PD0", 0},
    {"orange", "PD1", 1},
    {"green",  "PD2", 2},
    {"start",  "PD3", 3},
};

static bool ledOn[LED_COUNT];

static void led_set(int i, bool on)
{
    ledOn[i] = on;
    bool high = LED_ACTIVE_HIGH ? on : !on;
    GPIOD->BSRR = high ? (1u << leds[i].pin) : (1u << (leds[i].pin + 16));
}

/* ---------------------------------------------------------------- UART */

void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;

    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        /* Reading SR then DR clears RXNE and the error flags. A byte with
         * an error flag (bad baud rate, noise) is kept anyway, so you see
         * the garbage. If the ring is full the byte is lost. */
        uint8_t byte = (uint8_t)USART1->DR;
        uint32_t next = (rxHead + 1) & (RX_RING_SIZE - 1);

        if ((sr & USART_SR_RXNE) && next != rxTail) {
            rxRing[rxHead] = byte;
            rxHead = next;
        }
    }
}

static bool uart_read(uint8_t *byte)
{
    if (rxTail == rxHead)
        return false;
    *byte = rxRing[rxTail];
    rxTail = (rxTail + 1) & (RX_RING_SIZE - 1);
    return true;
}

static void uart_putc(char c)
{
    while (!(USART1->SR & USART_SR_TXE)) {
    }
    USART1->DR = (uint8_t)c;
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

static void uart_put_hex8(uint8_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    uart_putc(hex[v >> 4]);
    uart_putc(hex[v & 0xF]);
}

/* ---------------------------------------------------------------- set-up */

static void hw_init(void)
{
    SystemCoreClockUpdate();            /* 16 MHz (HSI) */

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIODEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;                 /* let the clock enable take effect */

    /* PD0..PD3: LEDs, push-pull outputs, all off. */
    for (int i = 0; i < LED_COUNT; i++)
        led_set(i, false);
    GPIOD->MODER = (GPIOD->MODER & ~(GPIO_MODER_MODER0 | GPIO_MODER_MODER1 |
                                     GPIO_MODER_MODER2 | GPIO_MODER_MODER3))
                 | (1u << GPIO_MODER_MODER0_Pos) | (1u << GPIO_MODER_MODER1_Pos)
                 | (1u << GPIO_MODER_MODER2_Pos) | (1u << GPIO_MODER_MODER3_Pos);

    /* PA9 = USART1_TX, PA10 = USART1_RX, both AF7. Pull-up on RX so an
     * unplugged adapter reads as an idle line, not noise. */
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10))
                  | (7u << GPIO_AFRH_AFSEL9_Pos) | (7u << GPIO_AFRH_AFSEL10_Pos);
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~GPIO_OSPEEDR_OSPEED9) | (1u << GPIO_OSPEEDR_OSPEED9_Pos);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPD10) | (1u << GPIO_PUPDR_PUPD10_Pos);
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10))
                 | (2u << GPIO_MODER_MODER9_Pos) | (2u << GPIO_MODER_MODER10_Pos);

    /* USART1 on APB2. With 16x oversampling, BRR = PCLK / baud, rounded
     * (mantissa and fraction packed together). 16 MHz / 115200 -> 139,
     * 0.08 % fast. */
    static const uint8_t apbShift[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    uint32_t pclk2 = SystemCoreClock >> apbShift[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos];
    USART1->CR1 = 0;
    USART1->CR2 = 0;                    /* 1 stop bit */
    USART1->CR3 = 0;
    USART1->BRR = (pclk2 + BAUD / 2) / BAUD;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;   /* 8N1 */

    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
}

/* ---------------------------------------------------------------- commands */

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

/* Takes the next space-separated word from *s (length *len). Sets *word and
 * *wordLen; leaves *s just after the word. Returns false if none left. */
static bool next_word(const char **s, size_t *len, const char **word, size_t *wordLen)
{
    while (*len && **s == ' ') {
        (*s)++;
        (*len)--;
    }
    if (*len == 0)
        return false;
    *word = *s;
    while (*len && **s != ' ') {
        (*s)++;
        (*len)--;
    }
    *wordLen = (size_t)(*s - *word);
    return true;
}

static void print_help(void)
{
    uart_puts("Commands:\r\n"
              "  led on | led off | led toggle | led status    all four LEDs\r\n"
              "  led <colour> on | off | toggle | status       one LED\r\n"
              "      colours: red, orange, green, start\r\n"
              "  echo <text>      sends back everything after \"echo \"\r\n"
              "  help\r\n");
}

static void print_led(int i)
{
    uart_puts(leds[i].name);
    uart_puts(" (");
    uart_puts(leds[i].pinName);
    uart_puts(ledOn[i] ? "): on\r\n" : "): off\r\n");
}

static void led_usage(void)
{
    uart_puts("usage: led [red|orange|green|start] on | off | toggle | status\r\n");
}

static void cmd_led(const char *args, size_t len)
{
    const char *w, *extra;
    size_t wl, extraLen;
    int first = 0, last = LED_COUNT - 1;    /* all four unless a colour is given */

    if (!next_word(&args, &len, &w, &wl)) {
        led_usage();
        return;
    }
    for (int i = 0; i < LED_COUNT; i++) {
        if (word_is(w, wl, leds[i].name)) {
            first = last = i;
            if (!next_word(&args, &len, &w, &wl)) {
                led_usage();
                return;
            }
            break;
        }
    }
    if (next_word(&args, &len, &extra, &extraLen)) {
        led_usage();
        return;
    }

    if (word_is(w, wl, "on")) {
        for (int i = first; i <= last; i++)
            led_set(i, true);
    } else if (word_is(w, wl, "off")) {
        for (int i = first; i <= last; i++)
            led_set(i, false);
    } else if (word_is(w, wl, "toggle")) {
        for (int i = first; i <= last; i++)
            led_set(i, !ledOn[i]);
    } else if (!word_is(w, wl, "status")) {
        led_usage();
        return;
    }
    for (int i = first; i <= last; i++)
        print_led(i);
}

static void run_line(const char *line, size_t len)
{
    const char *cmd;
    size_t cmdLen;

    if (!next_word(&line, &len, &cmd, &cmdLen))
        return;                         /* blank line */

    if (word_is(cmd, cmdLen, "echo")) {
        /* Everything after the single space that follows "echo", spaces
         * and all. Plain "echo" sends back an empty line. */
        if (len) {
            line++;
            len--;
        }
        uart_write(line, len);
        uart_puts("\r\n");
    } else if (word_is(cmd, cmdLen, "led")) {
        cmd_led(line, len);
    } else if (word_is(cmd, cmdLen, "help")) {
        print_help();
    } else {
        uart_puts("unknown command \"");
        uart_write(cmd, cmdLen);
        uart_puts("\", type help\r\n");
    }
}

/* ---------------------------------------------------------------- main */

int main(void)
{
    hw_init();

    uart_puts("\r\n\r\n=== PCB UART test: USART1 PA9 TX / PA10 RX (J7), 115200 8N1 ===\r\n");
    print_help();
    uart_puts("> ");

    char line[LINE_MAX];
    size_t len = 0;
    bool overflow = false;
    uint8_t prev = 0;

    for (;;) {
        uint8_t c;
        if (!uart_read(&c))
            continue;

        if (c == '\r' || c == '\n') {
            if (c == '\n' && prev == '\r') {    /* second half of CR LF */
                prev = c;
                continue;
            }
            uart_puts("\r\n");
            if (overflow)
                uart_puts("line too long (max 80 characters), ignored\r\n");
            else
                run_line(line, len);
            len = 0;
            overflow = false;
            uart_puts("> ");
        } else if (c == 0x08 || c == 0x7F) {    /* Backspace / Delete */
            if (len > 0) {
                len--;
                uart_puts("\b \b");
            }
        } else if (c >= 0x20 && c < 0x7F) {
            uart_putc((char)c);
            if (len < LINE_MAX)
                line[len++] = (char)c;
            else
                overflow = true;
        } else {                                /* show control bytes as <hex> */
            uart_putc('<');
            uart_put_hex8(c);
            uart_putc('>');
        }
        prev = c;
    }
}

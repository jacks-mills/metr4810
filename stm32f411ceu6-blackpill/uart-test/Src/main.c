/**
 * @file    main.c
 * @brief   UART terminal test for the Blackpill (STM32F411CEU6).
 *
 * Checks the USB-serial adapter and terminal program before they're used
 * with robot on the PCB. Same port and settings as the PCB terminal:
 * USART1, PA9 = TX, PA10 = RX, 115200 8N1.
 *
 * Commands (any case), one per line, ended by Enter (CR, LF or CR LF):
 *   led on | led off | led toggle | led status
 *   echo <text>        sends back everything after "echo "
 *   help
 *
 * Characters are echoed as you type them (terminal programs like picocom
 * don't show what you type), with Backspace working. Control bytes show as
 * <hex>, e.g. an arrow key as <1B>[A.
 *
 * LED: the Blackpill's blue LED on PC13, lit when the pin is low.
 *
 * Clock: left at the reset default, HSI 16 MHz with no PLL, so the test
 * doesn't depend on the crystal. USART1 runs from APB2 = 16 MHz.
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

/* ---------------------------------------------------------------- state */

static volatile uint8_t  rxRing[RX_RING_SIZE];
static volatile uint32_t rxHead;            /* written by the interrupt */
static uint32_t          rxTail;            /* written by main */

static bool ledOn;

/* ---------------------------------------------------------------- LED */

static void led_set(bool on)
{
    ledOn = on;
    if (on)
        GPIOC->BSRR = GPIO_BSRR_BR13;   /* low = lit */
    else
        GPIOC->BSRR = GPIO_BSRR_BS13;
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

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;                 /* let the clock enable take effect */

    /* PC13: LED, push-pull output, start off (high). */
    GPIOC->BSRR = GPIO_BSRR_BS13;
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER13) | (1u << GPIO_MODER_MODER13_Pos);

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
              "  led on | led off | led toggle | led status\r\n"
              "  echo <text>      sends back everything after \"echo \"\r\n"
              "  help\r\n");
}

static void print_led(void)
{
    uart_puts(ledOn ? "LED on\r\n" : "LED off\r\n");
}

static void cmd_led(const char *args, size_t len)
{
    const char *w, *extra;
    size_t wl, extraLen;

    if (!next_word(&args, &len, &w, &wl) || next_word(&args, &len, &extra, &extraLen)) {
        uart_puts("usage: led on | off | toggle | status\r\n");
        return;
    }
    if (word_is(w, wl, "on"))
        led_set(true);
    else if (word_is(w, wl, "off"))
        led_set(false);
    else if (word_is(w, wl, "toggle"))
        led_set(!ledOn);
    else if (!word_is(w, wl, "status")) {
        uart_puts("usage: led on | off | toggle | status\r\n");
        return;
    }
    print_led();
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
    led_set(false);

    uart_puts("\r\n\r\n=== Blackpill UART test: USART1 PA9 TX / PA10 RX, 115200 8N1 ===\r\n");
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

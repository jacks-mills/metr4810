/**
 * @file    servo_uart.c
 * @brief   USART2 single-wire (half-duplex) link to the ST3215 on PA2.
 *
 * Set up here, at the register level, rather than in CubeMX, so leave PA2 and
 * USART2 unassigned in CubeMX. On the PCB, SERVO_SIG is also PA2 / USART2_TX,
 * so this file carries over unchanged.
 *
 * Wiring: PA2 is open-drain, so the line needs a pull-up to 3.3 V. The
 * internal one (~40 kohm) is enabled, but at 1 Mbaud add an external 1-2.2
 * kohm resistor from the servo's signal wire to 3.3 V.
 *
 * Transfers are polled with interrupts masked (a FreeRTOS critical section):
 * at 1 Mbaud a byte arrives every 10 us and the servo answers within
 * microseconds of our last byte, so nothing may delay the reads. That holds
 * off the terminal and limit switch interrupts for ~0.3 ms per transfer (up
 * to ~1.5 ms if the servo doesn't answer). A limit switch pressed meanwhile
 * still cuts the drive motor, just that much later. (USB buffers what's
 * typed meanwhile, and the PCB's USART1 receives by DMA, so no keys are lost.)
 */
#include "config.h"
#if !HW_SIMULATED

#include <string.h>

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "servo_uart.h"

#define FIRST_BYTE_TIMEOUT_US   1000    /* servo must start answering by then */
#define GAP_TIMEOUT_US          200     /* ...and not pause longer than this mid-reply */
#define MAX_FRAME               64

static uint32_t cyclesPerUs;

void servo_uart_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef pin = {
        .Pin = GPIO_PIN_2,
        .Mode = GPIO_MODE_AF_OD,            /* both ends share the wire */
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF7_USART2,
    };
    HAL_GPIO_Init(GPIOA, &pin);

    USART2->CR1 = 0;
    USART2->CR2 = 0;                                    /* 1 stop bit */
    USART2->CR3 = USART_CR3_HDSEL;                      /* single-wire half duplex */
    USART2->BRR = (HAL_RCC_GetPCLK1Freq() + SERVO_BAUD / 2) / SERVO_BAUD;   /* 16x oversampling */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;              /* 8N1 */

    /* cycle counter for microsecond timeouts with interrupts masked */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cyclesPerUs = SystemCoreClock / 1000000;
}

static void drain(void)
{
    /* reading SR then DR clears RXNE and any overrun/noise/framing flags */
    while (USART2->SR & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE))
        (void)USART2->DR;
}

size_t servo_uart_transfer(const uint8_t *tx, size_t txLen, uint8_t *rx, size_t rxSize)
{
    uint8_t buf[MAX_FRAME];
    size_t want = txLen + rxSize;           /* echo + reply */
    size_t got = 0;

    if (want > sizeof buf)
        return 0;

    drain();
    taskENTER_CRITICAL();

    /* send, keeping up with the echo as it comes back */
    uint32_t txStart = DWT->CYCCNT;
    for (size_t sent = 0; sent < txLen; ) {
        if (DWT->CYCCNT - txStart > FIRST_BYTE_TIMEOUT_US * cyclesPerUs)
            break;                      /* never stay here with interrupts masked */
        uint32_t sr = USART2->SR;
        if (sr & USART_SR_TXE)
            USART2->DR = tx[sent++];
        if ((sr & USART_SR_RXNE) && got < want)
            buf[got++] = (uint8_t)USART2->DR;
    }

    /* the rest of the echo, then the reply */
    uint32_t last = DWT->CYCCNT;
    uint32_t timeout = FIRST_BYTE_TIMEOUT_US * cyclesPerUs;
    while (got < want) {
        if (USART2->SR & USART_SR_RXNE) {
            buf[got++] = (uint8_t)USART2->DR;
            last = DWT->CYCCNT;
            if (got > txLen)
                timeout = GAP_TIMEOUT_US * cyclesPerUs;
        } else if (DWT->CYCCNT - last > timeout) {
            break;
        }
    }

    taskEXIT_CRITICAL();

    /* strip our own bytes if they came back */
    size_t start = (got >= txLen && memcmp(buf, tx, txLen) == 0) ? txLen : 0;
    size_t n = got - start;
    if (n > rxSize)
        n = rxSize;
    memcpy(rx, buf + start, n);
    return n;
}

void servo_uart_delay_ms(uint32_t ms)
{
    osDelay(ms);
}

#endif /* !HW_SIMULATED */

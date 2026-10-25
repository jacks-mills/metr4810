/**
 * @file    servo_uart.c
 * @brief   The servo's one-wire bus: USART2 in half-duplex mode on PA2,
 *          1 Mbaud 8N1, register level.
 *
 * Sending: the request goes out byte after byte with interrupts off (about
 * 10 us a byte), with PA2 driving the line both ways (push-pull) for clean
 * edges. Then PA2 lets go of the line (open-drain, pull-up) and the
 * receiver is turned on, so our own bytes aren't read back.
 *
 * Receiving: DMA1 stream 5 (channel 4, USART2_RX) copies the reply into a
 * buffer, so a USB interrupt can't make us miss a byte. The reply is done
 * when the line has been quiet for REPLY_GAP_US after the last byte.
 *
 * Timing uses the core's cycle counter (DWT->CYCCNT).
 */
#include "main.h"
#include "servo.h"

#define SERVO_BAUD          1000000u
#define SERVO_PIN           2u          /* PA2, USART2_TX (AF7) */
#define REPLY_GAP_US        100u        /* quiet this long after a byte: reply finished */

static uint8_t rxDma[64];

static uint32_t us_to_cycles(uint32_t us)
{
    return us * (SystemCoreClock / 1000000u);
}

void servo_port_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->APB1ENR;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     /* cycle counter on */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* USART2: 8N1, half-duplex (TX pin only), receive by DMA. The receiver
     * is only on while waiting for a reply. */
    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = USART_CR3_HDSEL | USART_CR3_DMAR;
    USART2->BRR = (HAL_RCC_GetPCLK1Freq() + SERVO_BAUD / 2u) / SERVO_BAUD;   /* 48 MHz / 1 M = 48 */
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE;

    /* PA2: AF7, open-drain with pull-up while idle, high speed. Set up after
     * the USART so the line goes straight to idle (high). */
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFu << (4u * SERVO_PIN))) | (7u << (4u * SERVO_PIN));
    GPIOA->OTYPER |= 1u << SERVO_PIN;
    GPIOA->OSPEEDR |= 3u << (2u * SERVO_PIN);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3u << (2u * SERVO_PIN))) | (1u << (2u * SERVO_PIN));
    GPIOA->MODER = (GPIOA->MODER & ~(3u << (2u * SERVO_PIN))) | (2u << (2u * SERVO_PIN));

    DMA1_Stream5->CR = 0;
    while (DMA1_Stream5->CR & DMA_SxCR_EN) {
    }
    DMA1_Stream5->PAR = (uint32_t)&USART2->DR;
}

static void rx_stop(void)
{
    USART2->CR1 &= ~USART_CR1_RE;
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN) {
    }
    DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 |
                  DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5;
    (void)USART2->SR;                   /* clear any overrun / framing flags */
    (void)USART2->DR;
}

uint32_t servo_port_exchange(const uint8_t *tx, uint32_t txLen, uint8_t *rx, uint32_t rxMax,
                             uint32_t timeoutUs)
{
    if (rxMax > sizeof rxDma)
        rxMax = sizeof rxDma;
    rx_stop();

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    GPIOA->OTYPER &= ~(1u << SERVO_PIN);            /* drive the line while sending */
    for (uint32_t i = 0; i < txLen; i++) {
        while (!(USART2->SR & USART_SR_TXE)) {
        }
        USART2->DR = tx[i];
    }
    while (!(USART2->SR & USART_SR_TC)) {
    }
    GPIOA->OTYPER |= 1u << SERVO_PIN;               /* let go: the servo answers on it */

    DMA1_Stream5->M0AR = (uint32_t)rxDma;
    DMA1_Stream5->NDTR = rxMax;
    DMA1_Stream5->CR = (4u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_MINC | DMA_SxCR_EN;
    USART2->CR1 |= USART_CR1_RE;
    __set_PRIMASK(primask);

    uint32_t start = DWT->CYCCNT, lastChange = start, count = 0;
    for (;;) {
        uint32_t now = DWT->CYCCNT;
        uint32_t got = rxMax - DMA1_Stream5->NDTR;
        if (got != count) {
            count = got;
            lastChange = now;
        }
        if (count >= rxMax)
            break;
        if (count == 0 && now - start >= us_to_cycles(timeoutUs))
            break;
        if (count > 0 && now - lastChange >= us_to_cycles(REPLY_GAP_US))
            break;
    }
    rx_stop();

    for (uint32_t i = 0; i < count; i++)
        rx[i] = rxDma[i];
    return count;
}

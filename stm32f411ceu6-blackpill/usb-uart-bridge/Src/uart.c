/**
 * @file    uart.c
 * @brief   USART1 on PA9 (TX) and PA10 (RX), both directions by DMA.
 *
 * Same pins and DMA streams as the robot's PCB terminal (term_io_uart.c),
 * and set up the same way, at register level.
 *
 * RX: DMA2 stream 2 (channel 4) runs continuously into a 256-byte ring. The
 * half-full, full and IDLE-line interrupts hand the new bytes to the bridge,
 * so a byte waits at most one character time after a burst ends.
 *
 * TX: DMA2 stream 7 (channel 4) sends one block at a time, straight from the
 * bridge's FIFO. The main loop starts the next block once the stream stops.
 *
 * USART1 runs from PCLK2 (96 MHz), so 1500 to 6000000 baud is possible.
 */
#include "stm32f4xx.h"
#include "bridge.h"
#include "bridge_port.h"

#define RX_RING_SIZE        256u
#define UART_IRQ_PRIORITY   1u      /* below SysTick (0), above USB (5) */

/* Line errors seen, for the debugger (watch uartErrors). A growing
 * framingErrors count usually means the baud rates don't match. */
volatile struct {
    uint32_t framingErrors;
    uint32_t parityErrors;
    uint32_t noiseErrors;
    uint32_t overruns;
} uartErrors;

static uint8_t rxRing[RX_RING_SIZE];
static uint32_t rxNext;                     /* next ring index to hand over (interrupts only) */
static volatile uint8_t rxMask = 0xFFu;     /* 0x7F with 7 data bits + parity: drops the parity bit */
static uint32_t pclk2;

#define DMA_S2_FLAGS (DMA_LIFCR_CFEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2)
#define DMA_S7_FLAGS (DMA_HIFCR_CFEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTCIF7)

void uart_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;                     /* let the clocks start */

    pclk2 = SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos];

    /* RX: peripheral to memory, circular, interrupts at half and full */
    DMA2_Stream2->CR = 0;
    while (DMA2_Stream2->CR & DMA_SxCR_EN) {}
    DMA2->LIFCR = DMA_S2_FLAGS;
    DMA2_Stream2->PAR = (uint32_t)&USART1->DR;
    DMA2_Stream2->M0AR = (uint32_t)rxRing;
    DMA2_Stream2->NDTR = RX_RING_SIZE;
    DMA2_Stream2->FCR = 0;                  /* direct mode */
    DMA2_Stream2->CR = (4u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1 | DMA_SxCR_MINC
                     | DMA_SxCR_CIRC | DMA_SxCR_HTIE | DMA_SxCR_TCIE;
    DMA2_Stream2->CR |= DMA_SxCR_EN;
    rxNext = 0;

    /* TX: memory to peripheral, one block at a time (started by uart_tx_start) */
    DMA2_Stream7->CR = 0;
    while (DMA2_Stream7->CR & DMA_SxCR_EN) {}
    DMA2->HIFCR = DMA_S7_FLAGS;
    DMA2_Stream7->PAR = (uint32_t)&USART1->DR;
    DMA2_Stream7->FCR = 0;
    DMA2_Stream7->CR = (4u << DMA_SxCR_CHSEL_Pos) | DMA_SxCR_PL_1 | DMA_SxCR_MINC
                     | DMA_SxCR_DIR_0;

    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
    (void)uart_set_format(115200u, 8u, 0u, 0u);
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE | USART_CR1_UE;

    NVIC_SetPriority(USART1_IRQn, UART_IRQ_PRIORITY);
    NVIC_SetPriority(DMA2_Stream2_IRQn, UART_IRQ_PRIORITY);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    /* Pins last, once the USART is driving TX high (idle): no glitch that the
     * target could take for a start bit. AF7 = USART1. RX gets a pull-up so
     * it idles high with nothing connected. */
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(GPIO_AFRH_AFSEL9 | GPIO_AFRH_AFSEL10))
                  | (7u << GPIO_AFRH_AFSEL9_Pos) | (7u << GPIO_AFRH_AFSEL10_Pos);
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT9 | GPIO_OTYPER_OT10);
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(GPIO_OSPEEDR_OSPEED9 | GPIO_OSPEEDR_OSPEED10))
                   | (1u << GPIO_OSPEEDR_OSPEED9_Pos) | (1u << GPIO_OSPEEDR_OSPEED10_Pos);
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPD9 | GPIO_PUPDR_PUPD10))
                 | (1u << GPIO_PUPDR_PUPD10_Pos);
    GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10))
                 | (2u << GPIO_MODER_MODER9_Pos) | (2u << GPIO_MODER_MODER10_Pos);
}

bool uart_set_format(uint32_t baud, uint8_t dataBits, uint8_t parity, uint8_t stopBits)
{
    if (baud == 0u)
        return false;
    uint32_t brr = (pclk2 + baud / 2u) / baud;  /* 16x oversampling: mantissa and fraction in one */
    if (brr < 16u || brr > 0xFFFFu)
        return false;

    uint32_t cr1 = 0;
    uint8_t mask = 0xFFu;
    switch (parity) {
    case 0:                                 /* none: 8 data bits only (no 7N1 on this USART) */
        if (dataBits != 8u)
            return false;
        break;
    case 1:                                 /* odd */
    case 2:                                 /* even */
        cr1 = USART_CR1_PCE | (parity == 1u ? USART_CR1_PS : 0u);
        if (dataBits == 8u)
            cr1 |= USART_CR1_M;             /* 9-bit frame: 8 data + parity */
        else if (dataBits == 7u)
            mask = 0x7Fu;                   /* 8-bit frame: 7 data + parity */
        else
            return false;
        break;
    default:                                /* mark, space */
        return false;
    }

    uint32_t stop;
    switch (stopBits) {
    case 0: stop = 0u; break;                                   /* 1 */
    case 1: stop = USART_CR2_STOP_0 | USART_CR2_STOP_1; break;  /* 1.5 */
    case 2: stop = USART_CR2_STOP_1; break;                     /* 2 */
    default: return false;
    }

    /* Called only between bytes (see bridge.c), so the USART stays enabled. */
    USART1->BRR = brr;
    USART1->CR2 = (USART1->CR2 & ~USART_CR2_STOP) | stop;
    USART1->CR1 = (USART1->CR1 & ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS)) | cr1;
    rxMask = mask;
    return true;
}

/* ---- TX ------------------------------------------------------------------ */

bool uart_tx_busy(void)
{
    return (DMA2_Stream7->CR & DMA_SxCR_EN) != 0u;
}

bool uart_tx_complete(void)
{
    return !uart_tx_busy() && (USART1->SR & USART_SR_TC) != 0u;
}

void uart_tx_start(const uint8_t *data, uint32_t len)
{
    DMA2->HIFCR = DMA_S7_FLAGS;
    DMA2_Stream7->M0AR = (uint32_t)data;
    DMA2_Stream7->NDTR = len;
    USART1->SR = ~USART_SR_TC;              /* writing 0 clears TC; the other bits ignore it */
    DMA2_Stream7->CR |= DMA_SxCR_EN;
}

/* ---- RX ------------------------------------------------------------------ */

static void hand_over(uint32_t from, uint32_t to)
{
    if (rxMask != 0xFFu) {
        for (uint32_t i = from; i < to; i++)
            rxRing[i] &= rxMask;
    }
    bridge_uart_receive_isr(&rxRing[from], to - from);
}

/* Passes everything the DMA has written since last time to the bridge. */
static void rx_collect(void)
{
    uint32_t end = RX_RING_SIZE - DMA2_Stream2->NDTR;  /* where the DMA writes next */
    if (end >= RX_RING_SIZE)
        end = 0;

    if (end > rxNext) {
        hand_over(rxNext, end);
    } else if (end < rxNext) {              /* wrapped */
        hand_over(rxNext, RX_RING_SIZE);
        if (end > 0u)
            hand_over(0, end);
    }
    rxNext = end;
}

void USART1_IRQHandler(void)
{
    uint32_t sr = USART1->SR;

    if (sr & (USART_SR_IDLE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) {
        (void)USART1->DR;                   /* reading SR then DR clears these */
        if (sr & USART_SR_FE)
            uartErrors.framingErrors++;
        if (sr & USART_SR_PE)
            uartErrors.parityErrors++;
        if (sr & USART_SR_NE)
            uartErrors.noiseErrors++;
        if (sr & USART_SR_ORE)
            uartErrors.overruns++;
    }
    rx_collect();
}

void DMA2_Stream2_IRQHandler(void)
{
    DMA2->LIFCR = DMA_S2_FLAGS;
    rx_collect();
}

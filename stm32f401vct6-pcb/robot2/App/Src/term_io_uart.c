/**
 * @file    term_io_uart.c
 * @brief   Terminal transport over USART1 (PCB: PA9 TX, PA10 RX on J7),
 *          through a 3.3 V USB-serial adapter. 115200 8N1.
 *
 * Set up here at the register level, like the servo's USART2, so leave PA9,
 * PA10 and USART1 unassigned in CubeMX.
 *
 * Receive: DMA2 stream 2 copies every byte into a ring as it arrives, so
 * nothing is lost while the servo link has interrupts masked (up to ~1.5 ms,
 * and an arrow key is 3 bytes 87 us apart). When the line goes quiet, the
 * USART1 interrupt moves the new bytes into a stream buffer, which
 * term_io_read() waits on.
 *
 * Send: like the USB version. The next write waits until DMA2 stream 7 has
 * finished the previous chunk, copies up to TX_CHUNK_SIZE bytes into our
 * buffer and starts the DMA. Only the terminal task writes, so no lock.
 *
 * Connected: a serial line can't tell whether anyone is listening, so it
 * always counts as connected. The screen is drawn once at start-up; if you
 * open the terminal after that, press Ctrl-L to redraw it.
 */
#include <string.h>

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"
#include "term_io.h"

#define TERM_BAUD           115200
#define RX_RING_SIZE        256     /* DMA writes here, round and round */
#define RX_BUFFER_SIZE      256     /* bytes waiting for the terminal task */
#define TX_CHUNK_SIZE       1024    /* bytes per DMA transfer */
#define TX_TIMEOUT_MS       200     /* 1 KB takes 89 ms at 115200 */

/* Calls FreeRTOS from the interrupt, so it must not be more urgent (lower
 * number) than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5). */
#define UART_IRQ_PRIORITY   6

/* DMA2 request mapping (RM0368 table 28): USART1_RX = stream 2, USART1_TX =
 * stream 7, both channel 4. */
#define RX_DMA              DMA2_Stream2
#define TX_DMA              DMA2_Stream7
#define DMA_CHANNEL         (4u << DMA_SxCR_CHSEL_Pos)
#define RX_DMA_FLAGS        (DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | \
                             DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2)
#define TX_DMA_FLAGS        (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | \
                             DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7)

static uint8_t rxRing[RX_RING_SIZE];
static size_t rxTail;                           /* next ring byte to pass on (interrupt only) */

static uint8_t rxStorage[RX_BUFFER_SIZE + 1];   /* a stream buffer needs one spare byte */
static StaticStreamBuffer_t rxStreamStruct;
static StreamBufferHandle_t rxStream;

static uint8_t txBuffer[TX_CHUNK_SIZE];

static volatile bool connectEvent;

void term_io_init(void)
{
    /* trigger level 1: wake the reader on the first byte */
    rxStream = xStreamBufferCreateStatic(RX_BUFFER_SIZE, 1, rxStorage, &rxStreamStruct);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    GPIO_InitTypeDef pins = {
        .Pin = GPIO_PIN_9 | GPIO_PIN_10,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_PULLUP,            /* RX idles high with no adapter, instead of picking up noise */
        .Speed = GPIO_SPEED_FREQ_HIGH,
        .Alternate = GPIO_AF7_USART1,
    };
    HAL_GPIO_Init(GPIOA, &pins);

    USART1->CR1 = 0;
    USART1->CR2 = 0;                                    /* 1 stop bit */
    USART1->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
    USART1->BRR = (HAL_RCC_GetPCLK2Freq() + TERM_BAUD / 2) / TERM_BAUD;    /* 16x oversampling */

    /* receive: peripheral to memory, circular, byte by byte */
    RX_DMA->CR = 0;
    while (RX_DMA->CR & DMA_SxCR_EN) {
    }
    DMA2->LIFCR = RX_DMA_FLAGS;
    RX_DMA->PAR = (uint32_t)&USART1->DR;
    RX_DMA->M0AR = (uint32_t)rxRing;
    RX_DMA->NDTR = RX_RING_SIZE;
    RX_DMA->FCR = 0;                                    /* direct mode */
    RX_DMA->CR = DMA_CHANNEL | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_EN;

    /* send: memory to peripheral, started by term_io_write() */
    TX_DMA->CR = 0;
    while (TX_DMA->CR & DMA_SxCR_EN) {
    }
    DMA2->HIFCR = TX_DMA_FLAGS;
    TX_DMA->PAR = (uint32_t)&USART1->DR;
    TX_DMA->FCR = 0;
    TX_DMA->CR = DMA_CHANNEL | DMA_SxCR_MINC | DMA_SxCR_DIR_0;

    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_IDLEIE | USART_CR1_UE;   /* 8N1 */

    HAL_NVIC_SetPriority(USART1_IRQn, UART_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    connectEvent = true;                /* draw the screen once at start-up */
}

/* ---- interrupt ------------------------------------------------------------ */

/* Passes the bytes the DMA has written since last time to the stream buffer. */
static void rx_collect(BaseType_t *woken)
{
    size_t head = (RX_RING_SIZE - RX_DMA->NDTR) % RX_RING_SIZE;

    while (rxTail != head) {
        size_t end = head > rxTail ? head : RX_RING_SIZE;  /* up to the head, or the end of the ring */
        (void)xStreamBufferSendFromISR(rxStream, &rxRing[rxTail], end - rxTail, woken);  /* drops what doesn't fit */
        rxTail = end % RX_RING_SIZE;
    }
}

/* Not generated by CubeMX: leave the USART1 interrupt off there. */
void USART1_IRQHandler(void)
{
    BaseType_t woken = pdFALSE;

    if (USART1->SR & USART_SR_IDLE) {
        (void)USART1->DR;               /* SR then DR clears IDLE; the DMA already has the data */
        rx_collect(&woken);
    }
    portYIELD_FROM_ISR(woken);
}

/* ---- terminal task -------------------------------------------------------- */

bool term_io_connected(void)
{
    return true;
}

bool term_io_take_connect_event(void)
{
    if (!connectEvent)
        return false;
    connectEvent = false;
    return true;
}

size_t term_io_read(char *buf, size_t size, uint32_t timeoutTicks)
{
    return xStreamBufferReceive(rxStream, buf, size, timeoutTicks);
}

/* Waits for the previous transfer to finish. False if it didn't in time. */
static bool wait_tx_idle(void)
{
    uint32_t start = osKernelGetTickCount();

    while (TX_DMA->CR & DMA_SxCR_EN) {          /* the DMA clears EN when it's done */
        if (osKernelGetTickCount() - start >= TX_TIMEOUT_MS)
            return false;
        osDelay(1);
    }
    return true;
}

void term_io_write(const char *data, size_t len)
{
    while (len > 0) {
        if (!wait_tx_idle())
            return;                     /* shouldn't happen: drop the rest */

        size_t n = len < sizeof txBuffer ? len : sizeof txBuffer;
        memcpy(txBuffer, data, n);      /* sent from here after we return */
        DMA2->HIFCR = TX_DMA_FLAGS;     /* a stream won't start with old flags set */
        TX_DMA->M0AR = (uint32_t)txBuffer;
        TX_DMA->NDTR = n;
        TX_DMA->CR |= DMA_SxCR_EN;
        data += n;
        len -= n;
    }
}

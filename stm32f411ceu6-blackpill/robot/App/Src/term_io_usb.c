/**
 * @file    term_io_usb.c
 * @brief   Terminal transport over USB CDC (the Blackpill's USB-C port).
 *
 * Receive: the USB interrupt puts typed bytes into a stream buffer, and
 * term_io_read() waits on it.
 *
 * Send: CDC_Transmit_FS() sends from our buffer in the background; the next
 * write waits until it has finished. Only the terminal task writes, so there
 * is no lock.
 *
 * Connected: a terminal program raises DTR when it opens the port and drops
 * it when it closes. Some don't raise DTR, so receiving anything also counts.
 * Nothing is sent while no terminal is open.
 */
#include <string.h>

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "usbd_cdc_if.h"
#include "term_io.h"
#include "term_io_usb.h"

#define RX_BUFFER_SIZE      256     /* typed bytes waiting for the terminal task */
#define TX_CHUNK_SIZE       1024    /* bytes per USB transfer */
#define TX_TIMEOUT_MS       50      /* give up if the PC doesn't take a transfer */

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t rxStorage[RX_BUFFER_SIZE + 1];  /* a stream buffer needs one spare byte */
static StaticStreamBuffer_t rxStreamStruct;
static StreamBufferHandle_t rxStream;

static uint8_t txBuffer[TX_CHUNK_SIZE];

static volatile bool portOpen;
static volatile bool connectEvent;

void term_io_init(void)
{
    /* trigger level 1: wake the reader on the first byte */
    rxStream = xStreamBufferCreateStatic(RX_BUFFER_SIZE, 1, rxStorage, &rxStreamStruct);
}

/* ---- called from the USB interrupt -------------------------------------- */

static void set_port_open(bool open)
{
    if (open && !portOpen)
        connectEvent = true;
    portOpen = open;
}

void term_io_usb_reset_isr(void)
{
    set_port_open(false);
}

void term_io_usb_line_state_isr(bool dtr)
{
    set_port_open(dtr);
}

void term_io_usb_receive_isr(const uint8_t *data, uint32_t len)
{
    if (!portOpen)
        set_port_open(true);
    if (rxStream == NULL || len == 0)
        return;

    BaseType_t woken = pdFALSE;
    (void)xStreamBufferSendFromISR(rxStream, data, len, &woken);  /* drops what doesn't fit */
    portYIELD_FROM_ISR(woken);
}

/* ---- terminal task ------------------------------------------------------- */

bool term_io_connected(void)
{
    return portOpen && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED;
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

    for (;;) {
        USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
        if (hcdc == NULL)
            return false;
        if (hcdc->TxState == 0)
            return true;
        if (osKernelGetTickCount() - start >= TX_TIMEOUT_MS)
            return false;
        osDelay(1);
    }
}

void term_io_write(const char *data, size_t len)
{
    while (len > 0 && term_io_connected()) {
        if (!wait_tx_idle())
            return;                     /* the PC isn't reading: drop the rest */

        size_t n = len < sizeof txBuffer ? len : sizeof txBuffer;
        memcpy(txBuffer, data, n);      /* sent from here after we return */
        if (CDC_Transmit_FS(txBuffer, (uint16_t)n) != USBD_OK)
            return;
        data += n;
        len -= n;
    }
}

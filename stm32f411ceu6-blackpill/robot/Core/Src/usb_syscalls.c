/**
 * @file    usb_syscalls.c
 * @brief   newlib syscall overrides that route stdio over USB CDC.
 *
 * CubeIDE's syscalls.c defines _read, _write, etc. as __attribute__((weak)).
 * Any function defined here replaces the weak version at link time, so
 * syscalls.c can be left exactly as generated.
 *
 *   _write  -> printf, puts, fwrite(stdout)...
 *   _read   -> scanf, getchar, fgets(stdin)...
 *
 * Receive path:
 *   USB IRQ -> CDC_Receive_FS -> usb_syscalls_rx_isr -> stream buffer
 *   task    -> getchar/scanf  -> _read -> stream buffer (blocks until data)
 *
 * Only ONE task should read stdin: a FreeRTOS stream buffer supports a single
 * reader. Any number of tasks can write.
 */
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include "main.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "usbd_cdc_if.h"
#include "usb_syscalls.h"

/* Bytes held between the USB interrupt and _read. Anything received while
 * it's full is dropped. */
#define USB_RX_BUFFER_SIZE  256

/* 1: _read echoes typed characters back, so you can see what you type in
 * terminals without local echo (picocom, screen, minicom). 0: no echo. */
#define USB_STDIO_ECHO      1

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t rxStorage[USB_RX_BUFFER_SIZE + 1];   /* stream buffer needs +1 */
static StaticStreamBuffer_t rxStreamStruct;
static StreamBufferHandle_t rxStream = NULL;

/* ------------------------------------------------------------------------- */
/* Setup                                                                      */
/* ------------------------------------------------------------------------- */

void usb_syscalls_init(void)
{
    /* Trigger level 1: wake the reader as soon as any byte arrives. */
    rxStream = xStreamBufferCreateStatic(USB_RX_BUFFER_SIZE, 1,
                                         rxStorage, &rxStreamStruct);
}

/* ------------------------------------------------------------------------- */
/* Output                                                                     */
/* ------------------------------------------------------------------------- */

int _write(int file, char *ptr, int len)
{
    (void)file;
    static uint8_t txbuf[APP_TX_DATA_SIZE];

    // Drop output if no host is connected/enumerated
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
        return len;

    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

    int sent = 0;
    while (sent < len) {
        int chunk = len - sent;
        if (chunk > APP_TX_DATA_SIZE) chunk = APP_TX_DATA_SIZE;

        // Wait for the previous transfer to finish (with timeout)
        uint32_t start = HAL_GetTick();
        while (hcdc->TxState != 0) {
            if (HAL_GetTick() - start > 50) return sent;
        }

        // CDC_Transmit_FS doesn't copy data, so use our own buffer
        memcpy(txbuf, ptr + sent, chunk);

        start = HAL_GetTick();
        while (CDC_Transmit_FS(txbuf, chunk) == USBD_BUSY) {
            if (HAL_GetTick() - start > 50) return sent;
        }
        sent += chunk;
    }
    return len;
}

/* ------------------------------------------------------------------------- */
/* Input                                                                      */
/* ------------------------------------------------------------------------- */

void usb_syscalls_rx_isr(const uint8_t *buf, uint32_t len)
{
    if (rxStream == NULL || len == 0)
        return;

    BaseType_t higherPriorityTaskWoken = pdFALSE;
    (void)xStreamBufferSendFromISR(rxStream, buf, len, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

/* Blocks until at least one byte is available, then returns what's there
 * (up to len). Terminals send '\r' (or "\r\n") for Enter, but fgets/scanf
 * look for '\n', so "\r" and "\r\n" are both turned into a single '\n'. */
int _read(int file, char *ptr, int len)
{
    static int lastWasCR = 0;

    if (file != STDIN_FILENO) {
        errno = EBADF;
        return -1;
    }
    if (rxStream == NULL) {
        errno = EIO;
        return -1;
    }
    if (len <= 0)
        return 0;

    int out = 0;
    while (out == 0) {   /* returning 0 would mean end-of-file, so keep waiting */
        size_t n = xStreamBufferReceive(rxStream, ptr, (size_t)len, portMAX_DELAY);

        for (size_t i = 0; i < n; i++) {
            char c = ptr[i];
            if (c == '\r') {
                c = '\n';
                lastWasCR = 1;
            } else if (c == '\n' && lastWasCR) {
                lastWasCR = 0;
                continue;               /* second half of "\r\n" */
            } else {
                lastWasCR = 0;
            }
            ptr[out++] = c;
        }
    }

#if USB_STDIO_ECHO
    for (int i = 0; i < out; i++) {
        if (ptr[i] == '\n')
            _write(STDOUT_FILENO, "\r\n", 2);
        else
            _write(STDOUT_FILENO, &ptr[i], 1);
    }
#endif

    return out;
}

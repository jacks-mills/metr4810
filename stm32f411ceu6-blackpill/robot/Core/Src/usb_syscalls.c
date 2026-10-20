/**
 * @file    usb_syscalls.c
 * @brief   newlib syscall overrides that route stdio over USB CDC.
 *
 * CubeIDE's syscalls.c defines _read, _write, etc. as __attribute__((weak)).
 * Any function defined here replaces the weak version at link time, so
 * syscalls.c can be left exactly as generated.
 *
 *   _write  -> printf, puts, fwrite(stdout)...    (implemented)
 *   _read   -> scanf, getchar, fgets(stdin)...    (not yet; see below)
 */
#include <string.h>

#include "main.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

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
/* Input (to do)                                                              */
/* ------------------------------------------------------------------------- */
/*
 * To add _read:
 *   1. In USB_DEVICE/App/usbd_cdc_if.c, inside CDC_Receive_FS's USER CODE
 *      section, push the received bytes (Buf, *Len) into a FreeRTOS stream
 *      buffer with xStreamBufferSendFromISR (it runs in the USB interrupt).
 *   2. Define _read here so it blocks on xStreamBufferReceive and returns
 *      however many bytes arrived (at least 1, at most len).
 *
 * int _read(int file, char *ptr, int len)
 * {
 *     ...
 * }
 */

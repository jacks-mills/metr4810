/**
 * @file    usbd_cdc_if.c
 * @brief   USB CDC (virtual COM port) interface, used as a text console.
 *
 * Started from usb-uart-bridge's copy. The callbacks run in the USB
 * interrupt; the vcp_* functions run in the main loop and mask the USB
 * interrupt while they use the USB library.
 *
 * Typed bytes: each OUT packet is copied into rxRing. If less than a packet
 * of room is left, the endpoint isn't re-armed (the PC waits) until
 * vcp_poll() finds room again, so a paste never loses anything.
 *
 * Text out: queued in txRing, sent up to TX_CHUNK at a time by vcp_poll().
 * Dropped while no terminal has the port open.
 */
#include <string.h>

#include "usbd_cdc_if.h"
#include "main.h"
#include "vcp.h"

#define RX_SIZE     1024u           /* power of two */
#define TX_SIZE     4096u           /* power of two */
#define TX_CHUNK    512u
#define TX_WAIT_MS  100u            /* if the PC stops reading, give up after this */

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

static uint8_t rxRing[RX_SIZE];
static volatile uint32_t rxHead;    /* USB interrupt */
static volatile uint32_t rxTail;    /* main loop */
static volatile bool rxPaused;

static uint8_t txRing[TX_SIZE];
static uint32_t txHead, txTail;     /* main loop only */
static uint8_t txChunk[TX_CHUNK];   /* the transfer in progress is sent from here */

static volatile bool portOpen, justOpened;
static uint8_t lineCoding[7] = {0x00, 0xC2, 0x01, 0x00, 0, 0, 8};  /* 115200 8N1, unused */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
    CDC_Init_FS,
    CDC_DeInit_FS,
    CDC_Control_FS,
    CDC_Receive_FS,
    CDC_TransmitCplt_FS,
};

/* ---- USB interrupt ------------------------------------------------------- */

static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, NULL, 0);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    portOpen = false;
    rxPaused = false;
    return USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
    portOpen = false;
    return USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    switch (cmd) {
    case CDC_SET_LINE_CODING:
        if (length >= sizeof lineCoding)
            memcpy(lineCoding, pbuf, sizeof lineCoding);
        break;

    case CDC_GET_LINE_CODING:
        if (length >= sizeof lineCoding)
            memcpy(pbuf, lineCoding, sizeof lineCoding);
        break;

    case CDC_SET_CONTROL_LINE_STATE: {
        /* bit 0 of wValue is DTR, raised while a terminal has the port open */
        bool dtr = (((USBD_SetupReqTypedef *)pbuf)->wValue & 0x01u) != 0u;
        if (dtr && !portOpen)
            justOpened = true;
        portOpen = dtr;
        break;
    }

    default:
        break;
    }
    return USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
    uint32_t head = rxHead;
    for (uint32_t i = 0; i < *Len && head - rxTail < RX_SIZE; i++)
        rxRing[head++ % RX_SIZE] = Buf[i];
    rxHead = head;
    if (!portOpen)                      /* some programs never raise DTR */
        justOpened = true;
    portOpen = true;

    if (RX_SIZE - (head - rxTail) >= APP_RX_DATA_SIZE) {
        USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    } else {
        rxPaused = true;                /* vcp_poll() re-arms once there's room */
    }
    return USBD_OK;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    UNUSED(Buf);
    UNUSED(Len);
    UNUSED(epnum);
    return USBD_OK;
}

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL)
        return USBD_FAIL;
    if (hcdc->TxState != 0u)
        return USBD_BUSY;
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
    return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}

/* ---- main loop ----------------------------------------------------------- */

static bool configured(void)
{
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceFS.pClassData != NULL;
}

bool vcp_open(void)
{
    return configured() && portOpen;
}

bool vcp_just_opened(void)
{
    if (!justOpened)
        return false;
    justOpened = false;
    return true;
}

int vcp_getc(void)
{
    if (rxTail == rxHead)
        return -1;
    int c = rxRing[rxTail % RX_SIZE];
    rxTail++;
    return c;
}

void vcp_poll(void)
{
    NVIC_DisableIRQ(OTG_FS_IRQn);
    if (rxPaused && RX_SIZE - (rxHead - rxTail) >= APP_RX_DATA_SIZE && hUsbDeviceFS.pClassData != NULL) {
        rxPaused = false;
        USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    }
    NVIC_EnableIRQ(OTG_FS_IRQn);

    if (!vcp_open()) {
        txTail = txHead;                /* nobody listening */
        return;
    }
    if (txHead == txTail)
        return;

    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc == NULL || hcdc->TxState != 0u)
        return;
    uint32_t n = txHead - txTail;
    if (n > TX_CHUNK)
        n = TX_CHUNK;
    for (uint32_t i = 0; i < n; i++)
        txChunk[i] = txRing[(txTail + i) % TX_SIZE];

    NVIC_DisableIRQ(OTG_FS_IRQn);
    uint8_t result = CDC_Transmit_FS(txChunk, (uint16_t)n);
    NVIC_EnableIRQ(OTG_FS_IRQn);
    if (result == USBD_OK)
        txTail += n;
}

void vcp_write(const char *s, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (txHead - txTail >= TX_SIZE) {
            uint32_t start = HAL_GetTick();
            while (txHead - txTail >= TX_SIZE && HAL_GetTick() - start < TX_WAIT_MS)
                vcp_poll();
            if (txHead - txTail >= TX_SIZE)
                return;                 /* the PC isn't reading: drop the rest */
        }
        if (!vcp_open())
            return;
        txRing[txHead % TX_SIZE] = (uint8_t)s[i];
        txHead++;
    }
}

void vcp_puts(const char *s)
{
    vcp_write(s, strlen(s));
}

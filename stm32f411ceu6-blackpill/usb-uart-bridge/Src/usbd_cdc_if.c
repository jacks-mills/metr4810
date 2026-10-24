/**
 * @file    usbd_cdc_if.c
 * @brief   USB CDC (virtual COM port) interface: connects ST's CDC class to
 *          the bridge.
 *
 * Started from the CubeMX file in robot (USB_DEVICE/App/usbd_cdc_if.c).
 * This project has no .ioc, so it's ordinary code now.
 *
 * The callbacks run in the USB interrupt. The vcp_* functions are called from
 * the main loop, and mask the USB interrupt while they use the USB library.
 */
#include "usbd_cdc_if.h"
#include "bridge.h"
#include "bridge_port.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/* OUT packets from the PC land here (one at a time, then copied away). */
static uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

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

/* The PC configured the device (after plugging in, or a USB reset). The CDC
 * class arms the OUT endpoint itself after this returns. */
static int8_t CDC_Init_FS(void)
{
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, NULL, 0);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    bridge_usb_reset_isr();
    return USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
    bridge_usb_reset_isr();
    return USBD_OK;
}

/* Class requests. For requests with data (line coding) pbuf is the data;
 * for the others it's the setup request itself. */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    switch (cmd) {
    case CDC_SET_LINE_CODING:
        if (length >= 7u)
            bridge_usb_set_line_coding_isr(pbuf);
        break;

    case CDC_GET_LINE_CODING:
        if (length >= 7u)
            bridge_usb_get_line_coding_isr(pbuf);
        break;

    case CDC_SET_CONTROL_LINE_STATE:
        /* bit 0 of wValue is DTR, raised while a terminal has the port open */
        bridge_usb_line_state_isr((((USBD_SetupReqTypedef *)pbuf)->wValue & 0x01u) != 0u);
        break;

    default:                /* encapsulated commands, comm features, break: ignored */
        break;
    }
    return USBD_OK;
}

/* One OUT packet from the PC. Not re-arming the endpoint holds the PC off
 * (it gets NAKs) until the bridge has room; vcp_rx_resume() then re-arms. */
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
    if (bridge_usb_receive_isr(Buf, *Len)) {
        USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    }
    return USBD_OK;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    UNUSED(Buf);
    UNUSED(Len);
    UNUSED(epnum);
    return USBD_OK;         /* the bridge polls vcp_tx_idle() instead */
}

/* ---- main loop ----------------------------------------------------------- */

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

bool vcp_configured(void)
{
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceFS.pClassData != NULL;
}

bool vcp_tx_idle(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    return hcdc != NULL && hcdc->TxState == 0u;
}

bool vcp_tx_start(const uint8_t *data, uint32_t len)
{
    NVIC_DisableIRQ(OTG_FS_IRQn);
    uint8_t result = CDC_Transmit_FS((uint8_t *)data, (uint16_t)len);
    NVIC_EnableIRQ(OTG_FS_IRQn);
    return result == USBD_OK;
}

void vcp_rx_resume(void)
{
    NVIC_DisableIRQ(OTG_FS_IRQn);
    if (hUsbDeviceFS.pClassData != NULL) {
        USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    }
    NVIC_EnableIRQ(OTG_FS_IRQn);
}

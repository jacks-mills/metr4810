/**
 * @file    usbd_cdc_if.h
 * @brief   USB CDC (virtual COM port) interface. Started from the CubeMX file.
 */
#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#include "usbd_cdc.h"

/* One full-speed OUT packet at a time. Sent data comes from the bridge's
 * own buffer, so there's no transmit buffer here. */
#define APP_RX_DATA_SIZE  CDC_DATA_FS_OUT_PACKET_SIZE

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

#endif /* __USBD_CDC_IF_H__ */

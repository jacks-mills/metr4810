/**
 * @file    bridge.c
 * @brief   Moves bytes between the USB virtual COM port and USART1.
 *
 * PC -> target: the USB interrupt copies each OUT packet (up to 64 bytes)
 * into toUart. The main loop sends toUart out of USART1 by DMA, straight
 * from the FIFO. When less than a packet of space is left, the OUT endpoint
 * isn't re-armed, so the PC is held off (USB NAKs) until the UART has caught
 * up. Nothing typed or pasted is lost.
 *
 * Target -> PC: the UART interrupt copies received bytes into toUsb, and the
 * main loop sends them whenever the USB IN endpoint is free. The target can't
 * be held off, so if the PC stops reading for long enough (4 KB is 0.35 s at
 * 115200) the newest bytes are dropped and counted. While no terminal has
 * the port open (or USB isn't connected), the newest KEEP_WHILE_CLOSED bytes
 * are kept and sent as soon as a terminal opens it. So a screen the target
 * drew at reset, before the terminal was opened, still arrives.
 *
 * Line coding: the baud rate, parity and stop bits the PC sets are applied
 * to USART1 once everything already queued for it has been sent.
 *
 * Each FIFO has one writer (an interrupt, which only moves head) and one
 * reader (the main loop, which only moves tail), so no locking is needed.
 * head and tail count bytes forever; the index is the count modulo the size.
 */
#include <string.h>

#include "bridge.h"
#include "bridge_port.h"

#define TO_UART_SIZE    2048u   /* PC -> target FIFO, power of two */
#define TO_USB_SIZE     4096u   /* target -> PC FIFO, power of two */
#define USB_PACKET      64u     /* full-speed bulk packet */
#define UART_CHUNK_MAX  512u    /* bytes per UART DMA block */
#define USB_CHUNK_MAX   1024u   /* bytes per USB IN transfer */
#define KEEP_WHILE_CLOSED 3072u /* newest target output kept for when a terminal opens */

#define LED_FLICKER_MS  40u     /* LED blinks off this long per burst of traffic */

/* 115200 baud, 1 stop bit, no parity, 8 data bits */
static const uint8_t DEFAULT_CODING[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 0, 8 };

volatile struct BridgeStats bridgeStats;

static uint8_t toUart[TO_UART_SIZE];
static volatile uint32_t toUartHead;        /* written by the USB interrupt */
static volatile uint32_t toUartTail;        /* written by the main loop */
static uint32_t uartTxLen;                  /* bytes in the running DMA block, 0 = none */

static uint8_t toUsb[TO_USB_SIZE];
static volatile uint32_t toUsbHead;         /* written by the UART interrupt */
static volatile uint32_t toUsbTail;         /* written by the main loop */
static uint8_t usbTxBuffer[USB_CHUNK_MAX];  /* the IN transfer is sent from here */

static volatile bool outPaused;             /* OUT endpoint left un-armed: toUart nearly full */
static volatile bool portOpen;              /* a terminal has the port open */

static volatile uint8_t hostCoding[7];      /* last line coding the PC set */
static volatile bool codingPending;         /* ...and not yet applied */

static uint32_t lastTraffic;                /* board_millis() of the last data moved */

void bridge_init(void)
{
    toUartHead = toUartTail = 0;
    uartTxLen = 0;
    toUsbHead = toUsbTail = 0;
    outPaused = false;
    portOpen = false;
    for (unsigned i = 0; i < sizeof DEFAULT_CODING; i++)
        hostCoding[i] = DEFAULT_CODING[i];
    codingPending = false;
    lastTraffic = 0;
    bridgeStats.toUartDropped = 0;
    bridgeStats.toUsbDropped = 0;
    bridgeStats.codingRejected = 0;
}

/* ---- called from the USB interrupt --------------------------------------- */

void bridge_usb_reset_isr(void)
{
    portOpen = false;
    outPaused = false;      /* the CDC class arms the OUT endpoint itself when configured */
}

void bridge_usb_line_state_isr(bool dtr)
{
    portOpen = dtr;
}

bool bridge_usb_receive_isr(const uint8_t *data, uint32_t len)
{
    portOpen = true;        /* some programs never raise DTR */

    uint32_t head = toUartHead;
    uint32_t space = TO_UART_SIZE - (head - toUartTail);
    if (len > space) {      /* only after a USB reset re-armed the endpoint early */
        bridgeStats.toUartDropped += len - space;
        len = space;
    }
    for (uint32_t i = 0; i < len; i++)
        toUart[(head + i) % TO_UART_SIZE] = data[i];
    toUartHead = head + len;

    if (space - len >= USB_PACKET)
        return true;
    outPaused = true;       /* bridge_poll() re-arms once the UART has made room */
    return false;
}

void bridge_usb_set_line_coding_isr(const uint8_t *coding)
{
    for (unsigned i = 0; i < sizeof hostCoding; i++)
        hostCoding[i] = coding[i];
    codingPending = true;
}

void bridge_usb_get_line_coding_isr(uint8_t *coding)
{
    for (unsigned i = 0; i < sizeof hostCoding; i++)
        coding[i] = hostCoding[i];
}

/* ---- called from the UART interrupt -------------------------------------- */

void bridge_uart_receive_isr(const uint8_t *data, uint32_t len)
{
    uint32_t head = toUsbHead;
    uint32_t space = TO_USB_SIZE - (head - toUsbTail);
    if (len > space) {
        bridgeStats.toUsbDropped += len - space;
        len = space;
    }
    for (uint32_t i = 0; i < len; i++)
        toUsb[(head + i) % TO_USB_SIZE] = data[i];
    toUsbHead = head + len;
}

/* ---- main loop ----------------------------------------------------------- */

static void apply_line_coding(void)
{
    uint8_t c[7];

    codingPending = false;
    for (unsigned i = 0; i < sizeof c; i++)
        c[i] = hostCoding[i];
    if (codingPending)
        return;             /* the PC changed it again while we copied: next time */

    uint32_t baud = (uint32_t)c[0] | (uint32_t)c[1] << 8 | (uint32_t)c[2] << 16 | (uint32_t)c[3] << 24;
    if (!uart_set_format(baud, c[6], c[5], c[4]))
        bridgeStats.codingRejected++;
}

/* PC -> target */
static void pump_to_uart(uint32_t now)
{
    if (uartTxLen != 0 && !uart_tx_busy()) {
        toUartTail += uartTxLen;            /* that block has gone */
        uartTxLen = 0;
    }

    if (uartTxLen == 0) {
        if (codingPending) {
            /* change the format between bytes, after the last one has gone */
            if (uart_tx_complete())
                apply_line_coding();
        } else {
            uint32_t tail = toUartTail;
            uint32_t waiting = toUartHead - tail;
            if (waiting != 0) {
                uint32_t index = tail % TO_UART_SIZE;
                uint32_t n = waiting;
                if (n > TO_UART_SIZE - index)
                    n = TO_UART_SIZE - index;   /* up to the end of the buffer; the rest next time */
                if (n > UART_CHUNK_MAX)
                    n = UART_CHUNK_MAX;
                uartTxLen = n;
                uart_tx_start(&toUart[index], n);
                lastTraffic = now;
            }
        }
    }

    if (outPaused && TO_UART_SIZE - (toUartHead - toUartTail) >= USB_PACKET) {
        outPaused = false;
        vcp_rx_resume();
    }
}

/* target -> PC */
static void pump_to_usb(uint32_t now, bool usbUp)
{
    if (!usbUp || !portOpen) {
        /* nobody to send it to yet: keep only the newest bytes */
        uint32_t head = toUsbHead;
        if (head - toUsbTail > KEEP_WHILE_CLOSED)
            toUsbTail = head - KEEP_WHILE_CLOSED;
        return;
    }

    uint32_t tail = toUsbTail;
    uint32_t n = toUsbHead - tail;
    if (n == 0 || !vcp_tx_idle())
        return;
    if (n > USB_CHUNK_MAX)
        n = USB_CHUNK_MAX;

    uint32_t index = tail % TO_USB_SIZE;
    uint32_t first = TO_USB_SIZE - index;   /* bytes before the end of the buffer */
    if (first >= n) {
        memcpy(usbTxBuffer, &toUsb[index], n);
    } else {
        memcpy(usbTxBuffer, &toUsb[index], first);
        memcpy(usbTxBuffer + first, toUsb, n - first);
    }

    if (vcp_tx_start(usbTxBuffer, n)) {
        toUsbTail = tail + n;
        lastTraffic = now;
    }
}

/*
 * LED:  1 Hz blink         no USB connection (or not enumerated yet)
 *       short blip each s  connected, no terminal open
 *       on                 terminal open; flickers off with traffic
 */
static void update_led(uint32_t now, bool usbUp)
{
    bool on;

    if (!usbUp)
        on = now % 1000u < 500u;
    else if (!portOpen)
        on = now % 1000u < 50u;
    else if (now - lastTraffic < 2u * LED_FLICKER_MS)
        on = (now / LED_FLICKER_MS) % 2u == 0u;
    else
        on = true;

    board_led(on);
}

void bridge_poll(void)
{
    uint32_t now = board_millis();
    bool usbUp = vcp_configured();

    pump_to_uart(now);
    pump_to_usb(now, usbUp);
    update_led(now, usbUp);
}

/**
 * @file    bridge_test.c
 * @brief   PC test of bridge.c with a fake USB port and UART.
 *
 *     gcc -std=c11 -Wall -Wextra -I../Inc -o bridge_test bridge_test.c ../Src/bridge.c && ./bridge_test
 *
 * Checks that data gets through both ways in order and without loss, that
 * the PC is held off when the UART can't keep up, that nothing is sent to
 * the PC while no terminal is open, and that line codings are applied only
 * between bytes.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "bridge.h"
#include "bridge_port.h"

/* ---- fake board ---------------------------------------------------------- */

static uint32_t fakeMillis;
static bool ledOn;

uint32_t board_millis(void) { return fakeMillis; }
void board_led(bool on) { ledOn = on; }

/* ---- fake USB: a PC that writes packets and reads IN transfers ------------ */

static bool usbConfigured;
static bool outArmed;                   /* OUT endpoint ready for the next packet */
static int  inBusyPolls;                /* IN transfer completes after this many polls */
static bool hostReading = true;         /* false: the PC stops collecting IN transfers */
static uint8_t hostGot[100000];         /* everything the PC has received */
static uint32_t hostGotLen;
static uint32_t inTransfers;
static uint32_t maxInLen;

bool vcp_configured(void) { return usbConfigured; }
bool vcp_tx_idle(void) { return inBusyPolls == 0; }

bool vcp_tx_start(const uint8_t *data, uint32_t len)
{
    assert(inBusyPolls == 0);
    assert(len > 0 && len <= 1024);
    memcpy(&hostGot[hostGotLen], data, len);
    hostGotLen += len;
    inTransfers++;
    if (len > maxInLen)
        maxInLen = len;
    inBusyPolls = 2;
    return true;
}

void vcp_rx_resume(void)
{
    assert(!outArmed);                  /* never re-armed twice */
    outArmed = true;
}

/* The PC sends one packet if the endpoint is armed. Returns bytes sent. */
static uint32_t host_send(const uint8_t *data, uint32_t len)
{
    if (!outArmed)
        return 0;
    if (len > 64)
        len = 64;
    outArmed = bridge_usb_receive_isr(data, len);
    return len;
}

/* ---- fake UART: a target that echoes nothing, just records ---------------- */

static int uartBusyPolls;               /* DMA block finishes after this many polls */
static int uartBlockPolls = 3;          /* how slow the UART is */
static bool uartShifting;               /* last byte still leaving the pin */
static uint8_t targetGot[100000];
static uint32_t targetGotLen;
static uint32_t lastBaud = 115200;
static uint8_t lastBits = 8, lastParity, lastStop;
static uint32_t formatChanges;
static bool formatChangedMidByte;

bool uart_tx_busy(void) { return uartBusyPolls > 0; }
bool uart_tx_complete(void) { return uartBusyPolls == 0 && !uartShifting; }

void uart_tx_start(const uint8_t *data, uint32_t len)
{
    assert(uartBusyPolls == 0);
    assert(len > 0 && len <= 512);
    memcpy(&targetGot[targetGotLen], data, len);
    targetGotLen += len;
    uartBusyPolls = uartBlockPolls;
    uartShifting = true;
}

bool uart_set_format(uint32_t baud, uint8_t dataBits, uint8_t parity, uint8_t stopBits)
{
    if (uartBusyPolls > 0 || uartShifting)
        formatChangedMidByte = true;
    if (baud < 1500 || baud > 6000000 || parity > 2 || (parity == 0 && dataBits != 8))
        return false;
    lastBaud = baud;
    lastBits = dataBits;
    lastParity = parity;
    lastStop = stopBits;
    formatChanges++;
    return true;
}

void uart_init(void) {}

/* One main-loop pass, with the fakes' time moving on. */
static void poll(void)
{
    bridge_poll();
    fakeMillis++;
    if (inBusyPolls > 0 && hostReading)
        inBusyPolls--;
    if (uartBusyPolls > 0)
        uartBusyPolls--;
    else
        uartShifting = false;           /* one poll after the DMA stops, the line is idle */
}

static void reset_all(void)
{
    bridge_init();
    usbConfigured = true;
    outArmed = true;                    /* the CDC class arms OUT on configuration */
    bridge_usb_reset_isr();
    inBusyPolls = 0;
    hostReading = true;
    hostGotLen = 0;
    inTransfers = 0;
    maxInLen = 0;
    uartBusyPolls = 0;
    uartBlockPolls = 3;
    uartShifting = false;
    targetGotLen = 0;
    formatChanges = 0;
    formatChangedMidByte = false;
    lastBaud = 115200;
}

static uint8_t pattern(uint32_t i) { return (uint8_t)(i * 7u + (i >> 8)); }

/* ---- tests --------------------------------------------------------------- */

static void test_pc_to_target_with_flow_control(void)
{
    reset_all();
    uartBlockPolls = 40;                /* UART much slower than USB */
    enum { N = 20000 };
    static uint8_t src[N];
    for (uint32_t i = 0; i < N; i++)
        src[i] = pattern(i);

    uint32_t sent = 0, pausedSeen = 0;
    for (int loops = 0; targetGotLen < N && loops < 1000000; loops++) {
        if (sent < N) {
            uint32_t n = host_send(&src[sent], N - sent < 64 ? N - sent : 64);
            sent += n;
            if (n == 0)
                pausedSeen++;
        }
        poll();
    }
    assert(targetGotLen == N);
    assert(memcmp(targetGot, src, N) == 0);
    assert(pausedSeen > 0);             /* the PC really was held off */
    assert(bridgeStats.toUartDropped == 0);
    printf("ok  PC -> target: %u bytes in order, PC held off %u times\n", N, pausedSeen);
}

static void test_target_to_pc(void)
{
    reset_all();
    bridge_usb_line_state_isr(true);    /* terminal opened the port */
    enum { N = 30000 };
    static uint8_t src[N];
    for (uint32_t i = 0; i < N; i++)
        src[i] = pattern(i + 99);

    uint32_t given = 0;
    while (hostGotLen < N) {
        if (given < N) {                /* uneven bursts, like the UART interrupts */
            uint32_t n = 1 + given % 150;
            if (n > N - given)
                n = N - given;
            bridge_uart_receive_isr(&src[given], n);
            given += n;
        }
        poll();
    }
    assert(memcmp(hostGot, src, N) == 0);
    assert(bridgeStats.toUsbDropped == 0);
    printf("ok  target -> PC: %u bytes in order, %u IN transfers, largest %u\n",
           N, inTransfers, maxInLen);
}

static void test_wraparound_split_copy(void)
{
    reset_all();
    bridge_usb_line_state_isr(true);
    hostReading = false;                /* hold the IN endpoint busy... */
    uint8_t first[1] = { 0xAA };
    bridge_uart_receive_isr(first, 1);
    poll();                             /* ...with one transfer in flight */
    static uint8_t src[5000];
    for (uint32_t i = 0; i < sizeof src; i++)
        src[i] = pattern(i + 5);
    bridge_uart_receive_isr(src, 4000); /* fills toUsb to 4000 of 4096 */
    for (int i = 0; i < 10; i++)
        poll();
    hostReading = true;
    for (int i = 0; i < 50; i++)
        poll();
    bridge_uart_receive_isr(src + 4000, 1000);   /* this one wraps the FIFO */
    for (int i = 0; i < 50; i++)
        poll();
    assert(hostGotLen == 5001);
    assert(hostGot[0] == 0xAA);
    assert(memcmp(hostGot + 1, src, 5000) == 0);
    printf("ok  target -> PC across the end of the FIFO\n");
}

static void test_pc_stops_reading(void)
{
    reset_all();
    bridge_usb_line_state_isr(true);
    hostReading = false;
    static uint8_t src[6000];
    for (uint32_t i = 0; i < sizeof src; i++)
        src[i] = pattern(i);
    bridge_uart_receive_isr(src, 1);
    poll();                             /* one byte in flight, IN stuck */
    bridge_uart_receive_isr(src + 1, 5999);
    assert(bridgeStats.toUsbDropped == 5999 - 4096);
    hostReading = true;
    for (int i = 0; i < 100; i++)
        poll();
    assert(hostGotLen == 1 + 4096);
    assert(memcmp(hostGot, src, 4097) == 0);  /* oldest kept, newest dropped */
    printf("ok  PC stops reading: newest %u bytes dropped and counted\n", bridgeStats.toUsbDropped);
}

static void test_port_closed_keeps_newest(void)
{
    reset_all();

    /* the target draws its screen at reset, before the terminal is opened */
    static uint8_t screen[2500];
    for (uint32_t i = 0; i < sizeof screen; i++)
        screen[i] = pattern(i + 7);
    bridge_uart_receive_isr(screen, 1000);
    for (int i = 0; i < 20; i++)
        poll();
    bridge_uart_receive_isr(screen + 1000, 1500);
    for (int i = 0; i < 20; i++)
        poll();
    assert(hostGotLen == 0);            /* nobody listening yet */

    bridge_usb_line_state_isr(true);    /* terminal opened: the screen arrives */
    for (int i = 0; i < 50; i++)
        poll();
    assert(hostGotLen == sizeof screen && memcmp(hostGot, screen, sizeof screen) == 0);

    /* more than KEEP_WHILE_CLOSED while closed: only the newest 3072 kept */
    bridge_usb_line_state_isr(false);
    hostGotLen = 0;
    static uint8_t lots[63 * 128];      /* 8064 bytes, in DMA-sized chunks */
    for (uint32_t i = 0; i < sizeof lots; i++)
        lots[i] = pattern(i + 3);
    for (uint32_t done = 0; done < sizeof lots; done += 128) {   /* like the DMA interrupts */
        bridge_uart_receive_isr(lots + done, 128);
        poll();
    }
    bridge_usb_line_state_isr(true);
    for (int i = 0; i < 50; i++)
        poll();
    assert(hostGotLen == 3072 && memcmp(hostGot, lots + sizeof lots - 3072, 3072) == 0);
    assert(bridgeStats.toUsbDropped == 0);

    /* typing without DTR (some programs) still opens the port */
    bridge_usb_line_state_isr(false);
    hostGotLen = 0;
    host_send((const uint8_t *)"a", 1);
    bridge_uart_receive_isr((const uint8_t *)"b", 1);
    for (int i = 0; i < 20; i++)
        poll();
    assert(hostGotLen == 1 && hostGot[0] == 'b');

    /* unplugged, then plugged in and opened: the output in between arrives */
    usbConfigured = false;
    bridge_usb_reset_isr();
    bridge_uart_receive_isr((const uint8_t *)"boot", 4);
    poll();
    usbConfigured = true;
    for (int i = 0; i < 5; i++)
        poll();
    assert(hostGotLen == 1);
    bridge_usb_line_state_isr(true);
    for (int i = 0; i < 20; i++)
        poll();
    assert(hostGotLen == 5 && memcmp(hostGot + 1, "boot", 4) == 0);
    printf("ok  output while no terminal is open is kept (newest 3072 bytes) and sent on open\n");
}

static void test_line_coding(void)
{
    reset_all();
    uint8_t c[7];
    bridge_usb_get_line_coding_isr(c);
    assert(c[0] == 0x00 && c[1] == 0xC2 && c[2] == 0x01 && c[3] == 0 && c[6] == 8);   /* 115200 8N1 */

    /* queue data, then change to 9600 8E1 while it's being sent */
    uartBlockPolls = 10;
    uint8_t data[64];
    memset(data, 'd', sizeof data);
    host_send(data, 64);
    poll();                             /* first block started */
    host_send(data, 64);                /* more, sent after the change */
    const uint8_t c9600e[7] = { 0x80, 0x25, 0x00, 0x00, 0, 2, 8 };
    bridge_usb_set_line_coding_isr(c9600e);
    bridge_usb_get_line_coding_isr(c);
    assert(memcmp(c, c9600e, 7) == 0);  /* the PC reads back what it set */
    for (int i = 0; i < 100; i++)
        poll();
    assert(formatChanges == 1 && lastBaud == 9600 && lastParity == 2);
    assert(!formatChangedMidByte);
    assert(targetGotLen == 128);

    /* something USART1 can't do: kept the old format, counted */
    const uint8_t mark[7] = { 0x00, 0xC2, 0x01, 0x00, 0, 3, 8 };
    bridge_usb_set_line_coding_isr(mark);
    for (int i = 0; i < 10; i++)
        poll();
    assert(lastBaud == 9600 && bridgeStats.codingRejected == 1);
    printf("ok  line coding applied between bytes; unsupported one rejected\n");
}

static void test_usb_reset_while_paused(void)
{
    reset_all();
    uartBlockPolls = 1000000;           /* UART stuck: fill toUart completely */
    uint8_t data[64];
    memset(data, 'r', sizeof data);
    poll();
    while (host_send(data, 64) != 0) {}
    assert(!outArmed);
    /* PC resets the device: the CDC class re-arms OUT by itself */
    bridge_usb_reset_isr();
    outArmed = true;
    host_send(data, 64);                /* may not fit: dropped and counted, no crash */
    for (int i = 0; i < 5; i++)
        poll();
    printf("ok  USB reset while held off (dropped %u)\n", bridgeStats.toUartDropped);
}

static void test_led(void)
{
    reset_all();
    usbConfigured = false;
    fakeMillis = 100;  poll(); assert(ledOn);
    fakeMillis = 600;  poll(); assert(!ledOn);
    usbConfigured = true;
    fakeMillis = 1010; poll(); assert(ledOn);       /* blip */
    fakeMillis = 1200; poll(); assert(!ledOn);
    bridge_usb_line_state_isr(true);
    fakeMillis = 5000; poll(); assert(ledOn);       /* steady */
    printf("ok  LED patterns\n");
}

int main(void)
{
    test_pc_to_target_with_flow_control();
    test_target_to_pc();
    test_wraparound_split_copy();
    test_pc_stops_reading();
    test_port_closed_keeps_newest();
    test_line_coding();
    test_usb_reset_while_paused();
    test_led();
    printf("all passed\n");
    return 0;
}

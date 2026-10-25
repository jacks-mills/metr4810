/* Software UART bit logic (8N1, LSB first). TIM1 timing is in main.c. */
#ifndef SWUART_H
#define SWUART_H

#include <stdbool.h>
#include <stdint.h>

/* Bit 0: start (low), 1-8: data, 9: stop (high). */
static inline bool swu_tx_level(uint8_t byte, unsigned bit)
{
    if (bit == 0)
        return false;
    if (bit >= 9)
        return true;
    return (byte >> (bit - 1u)) & 1u;
}

/* Transmitter state */
struct SwuTx {
    uint8_t byte;               /* byte being sent */
    uint8_t bit;                /* bit whose edge has just happened; 10 = end of the stop bit */
};

enum SwuTxNext {
    SWU_TX_LOW,                 /* next edge: output low */
    SWU_TX_HIGH,                /* next edge: output high (or stays high) */
    SWU_TX_IDLE,                /* finished: nothing more to schedule */
};

/* Next level after bit t->bit starts; takes the next byte if there is one. */
static inline enum SwuTxNext swu_tx_after_edge(struct SwuTx *t, bool haveNext, uint8_t next, bool *took)
{
    *took = false;
    if (t->bit < 9) {
        t->bit++;
        return swu_tx_level(t->byte, t->bit) ? SWU_TX_HIGH : SWU_TX_LOW;
    }
    if (t->bit == 9) {                  /* stop bit has just started */
        if (haveNext) {
            t->byte = next;
            t->bit = 0;
            *took = true;
            return SWU_TX_LOW;          /* next start bit, at the end of this stop bit */
        }
        t->bit = 10;
        return SWU_TX_HIGH;             /* mark the end of the stop bit */
    }
    return SWU_TX_IDLE;                 /* stop bit has ended */
}

/* Receiver: sample 0 is the start bit, 1-8 data, 9 stop. */
struct SwuRx {
    uint8_t bit;                /* which sample comes next */
    uint8_t data;
};

enum SwuRxResult {
    SWU_MORE,                   /* keep sampling */
    SWU_BYTE,                   /* *byte is a good byte */
    SWU_FRAMING,                /* stop bit was low: wrong baud rate or a line fault */
    SWU_BREAK,                  /* all zeros and no stop bit: the line is held low */
    SWU_GLITCH,                 /* the start bit had gone by its middle: noise */
};

static inline enum SwuRxResult swu_rx_sample(struct SwuRx *r, bool level, uint8_t *byte)
{
    if (r->bit == 0) {
        if (level)
            return SWU_GLITCH;
        r->bit = 1;
        r->data = 0;
        return SWU_MORE;
    }
    if (r->bit <= 8) {
        if (level)
            r->data |= (uint8_t)(1u << (r->bit - 1u));
        r->bit++;
        return SWU_MORE;
    }
    r->bit = 0;
    *byte = r->data;
    if (level)
        return SWU_BYTE;
    return r->data == 0 ? SWU_BREAK : SWU_FRAMING;
}

#endif /* SWUART_H */

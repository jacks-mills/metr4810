/* Motor automatic mode: pulse, wait, ... pulse one way; rest; flip M; the
 * same back; and so on. SysTick calls autorun_tick() once per ms. */
#ifndef AUTORUN_H
#define AUTORUN_H

#include <stdbool.h>
#include <stdint.h>

struct AutoSettings {
    uint32_t pulseMs;           /* E on, each pulse: 1 or more */
    uint32_t waitMs;            /* E off after each pulse; 0 = pulses run together */
    uint32_t timeoutMs;         /* reverse once this long has gone by one way; 0 = never */
    uint32_t count;             /* pulses each way; 0 = no limit */
    uint32_t restMs;            /* E off before each reversal */
    uint32_t cycles;            /* there-and-back cycles, then stop; 0 = until stopped */
    bool firstDir;              /* M for the first way */
};

enum AutoStep { AUTO_PULSE, AUTO_WAIT, AUTO_REST };

struct AutoRun {
    enum AutoStep step;
    uint32_t leftMs;            /* still to go in this step */
    uint32_t wayMs;             /* since the first pulse this way began */
    uint32_t pulses;            /* pulses finished this way */
    uint32_t ways;              /* ways finished: 2 per cycle */
    uint32_t total;             /* pulses finished altogether */
};

enum AutoEvent {
    AUTO_NONE,
    AUTO_REVERSE,               /* the rest is over: flip M; a pulse starts now */
    AUTO_DONE,                  /* all the cycles are done: E off, stop */
};

/* The first pulse, about to begin. */
static inline void autorun_begin(struct AutoRun *r, const struct AutoSettings *s)
{
    r->step = AUTO_PULSE;
    r->leftMs = s->pulseMs;
    r->wayMs = 0;
    r->pulses = 0;
    r->ways = 0;
    r->total = 0;
}

static inline bool autorun_e_on(const struct AutoRun *r)
{
    return r->step == AUTO_PULSE;
}

/* Called as a pulse ends: is that the last one this way? */
static inline bool autorun_way_over(const struct AutoRun *r, const struct AutoSettings *s)
{
    if (s->count && r->pulses >= s->count)
        return true;
    return s->timeoutMs && (uint64_t)r->wayMs + s->waitMs >= s->timeoutMs;
}

/* One ms has gone by. Changed settings apply from the next step. */
static inline enum AutoEvent autorun_tick(struct AutoRun *r, const struct AutoSettings *s)
{
    if (r->step != AUTO_REST && r->wayMs != UINT32_MAX)
        r->wayMs++;
    if (r->leftMs > 1) {
        r->leftMs--;
        return AUTO_NONE;
    }

    switch (r->step) {
    case AUTO_PULSE:
        r->pulses++;
        r->total++;
        if (!autorun_way_over(r, s)) {
            if (s->waitMs) {
                r->step = AUTO_WAIT;
                r->leftMs = s->waitMs;
            } else {
                r->leftMs = s->pulseMs;     /* straight into the next pulse */
            }
            return AUTO_NONE;
        }
        r->ways++;
        if (s->cycles && r->ways >= 2u * s->cycles)
            return AUTO_DONE;
        r->step = AUTO_REST;
        r->leftMs = s->restMs;
        return AUTO_NONE;

    case AUTO_WAIT:
        r->step = AUTO_PULSE;
        r->leftMs = s->pulseMs;
        return AUTO_NONE;

    default:                                /* AUTO_REST */
        r->step = AUTO_PULSE;
        r->leftMs = s->pulseMs;
        r->wayMs = 0;
        r->pulses = 0;
        return AUTO_REVERSE;
    }
}

/* Pulses each way, or 0 if it never reverses (no timeout and no count). */
static inline uint32_t autorun_pulses_each_way(const struct AutoSettings *s)
{
    uint64_t n = UINT64_MAX;
    if (s->count)
        n = s->count;
    if (s->timeoutMs) {
        /* pulse k starts at k * (pulse + wait) */
        uint64_t period = (uint64_t)s->pulseMs + s->waitMs;
        uint64_t byTime = period ? (s->timeoutMs + period - 1u) / period : 1u;
        if (byTime < n)
            n = byTime;
    }
    return n == UINT64_MAX ? 0u : (uint32_t)n;
}

/* One way, first pulse start to last pulse end, in ms. */
static inline uint64_t autorun_way_ms(const struct AutoSettings *s)
{
    uint64_t n = autorun_pulses_each_way(s);
    return n ? n * s->pulseMs + (n - 1u) * s->waitMs : 0u;
}

#endif /* AUTORUN_H */

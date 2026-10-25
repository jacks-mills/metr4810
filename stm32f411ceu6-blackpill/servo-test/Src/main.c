/* Servo test for the Blackpill (STM32F411CEU6): a command prompt on the
 * USB-C port for the ST3215 bus servo on PA2 (USART2 half-duplex, 1 Mbaud).
 * Wiring: servo signal -> PA2 (1 kohm pull-up to 3.3 V), GND shared, servo
 * power from its own supply. Type help.
 *
 * AI-generated code: written with Claude Opus 5.5 (claude-opus-5-5), an AI
 * assistant by Anthropic, October 2026. */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "usb_device.h"
#include "servo.h"
#include "vcp.h"

#define LED_PIN         13u         /* PC13, lit when low */
#define LINE_MAX        80u
#define HISTORY_SIZE    16u
#define ESC_WAIT_MS     20u         /* lone Esc after this long = the Esc key */
#define SCAN_TIMEOUT_US 1000u       /* per id while scanning */

#define STEPS_PER_TURN  4096u
#define DEFAULT_SPEED   500u        /* steps/s (4096 steps = 360 deg) */
#define DEFAULT_ACC     50u         /* x100 steps/s^2 */

static uint8_t servoId = 1;
static uint32_t moveSpeed = DEFAULT_SPEED;
static uint32_t moveAcc = DEFAULT_ACC;

/* ---- Output ---- */

static void out(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static void out(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > 0)
        vcp_write(buf, (size_t)n < sizeof buf ? (size_t)n : sizeof buf - 1u);
}

/* Prints what went wrong with a servo exchange. True if it worked. */
static bool report(enum ServoResult r, uint8_t error)
{
    if (r == SERVO_NO_REPLY) {
        out("no reply from servo %u (wiring, power, id or baud rate?)\r\n", servoId);
        return false;
    }
    if (r == SERVO_BAD_REPLY) {
        out("bad reply from servo %u (%lu bytes:", servoId, (unsigned long)servoLastRxLen);
        for (uint32_t i = 0; i < servoLastRxLen && i < 24u; i++)
            out(" %02X", servoLastRx[i]);
        out(")\r\n");
        return false;
    }
    if (error)
        out("servo %u reports fault flags 0x%02X\r\n", servoId, error);
    return true;
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

/* Sign-magnitude: the given bit is the sign. */
static int32_t signed_mag(uint16_t v, unsigned signBit)
{
    int32_t mag = v & ((1u << signBit) - 1u);
    return (v & (1u << signBit)) ? -mag : mag;
}

/* Tenths of a degree from steps. */
static int32_t steps_to_decideg(int32_t steps)
{
    return steps * 3600 / (int32_t)STEPS_PER_TURN;
}

static void out_tenths(const char *before, int32_t tenths, const char *after)
{
    const char *sign = tenths < 0 ? "-" : "";
    if (tenths < 0)
        tenths = -tenths;
    out("%s%s%ld.%ld%s", before, sign, (long)(tenths / 10), (long)(tenths % 10), after);
}

/* ---- Servo commands ---- */

static bool write_bytes(uint8_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t err = 0;
    enum ServoResult r = servo_write(servoId, addr, data, len, &err);
    return report(r, err);
}

static bool write_byte(uint8_t addr, uint8_t value)
{
    return write_bytes(addr, &value, 1);
}

static bool read_bytes(uint8_t addr, uint8_t len, uint8_t *outBuf)
{
    uint8_t err = 0;
    enum ServoResult r = servo_read(servoId, addr, len, outBuf, &err);
    return report(r, err);
}

/* Torque on first (turning it on resets the goal), then the goal. */
static void move_to(uint32_t steps)
{
    if (steps >= STEPS_PER_TURN)
        steps = STEPS_PER_TURN - 1u;
    uint8_t torque;
    if (!read_bytes(REG_TORQUE, 1, &torque))
        return;
    if (torque != 1 && !write_byte(REG_TORQUE, 1))
        return;
    uint8_t goal[7] = {
        (uint8_t)moveAcc,                           /* 41 acceleration */
        (uint8_t)steps, (uint8_t)(steps >> 8),      /* 42 goal position */
        0, 0,                                       /* 44 goal time: unused */
        (uint8_t)moveSpeed, (uint8_t)(moveSpeed >> 8),  /* 46 goal speed */
    };
    if (!write_bytes(REG_ACC, goal, sizeof goal))
        return;
    out("moving to %lu", (unsigned long)steps);
    out_tenths(" (", steps_to_decideg((int32_t)steps), " deg)");
    out(" at %lu steps/s, acc %lu\r\n", (unsigned long)moveSpeed, (unsigned long)moveAcc);
}

/* Goal = where it is now. */
static void hold_here(void)
{
    uint8_t p[2];
    if (!read_bytes(REG_PRESENT_POS, 2, p))
        return;
    int32_t pos = signed_mag(get16(p), 15);
    if (pos < 0)
        pos = 0;
    uint8_t goal[2] = {(uint8_t)pos, (uint8_t)(pos >> 8)};
    if (write_bytes(REG_GOAL_POS, goal, 2))
        out("holding at %ld\r\n", (long)pos);
}

static void cmd_scan(void)
{
    out("scanning ids 0-253...\r\n");
    unsigned found = 0;
    for (unsigned id = 0; id <= 253u; id++) {
        uint8_t err = 0;
        vcp_poll();
        if (servo_ping((uint8_t)id, SCAN_TIMEOUT_US, &err) != SERVO_OK)
            continue;
        found++;
        uint8_t m[2];
        uint8_t err2 = 0;
        if (servo_read((uint8_t)id, REG_MODEL, 2, m, &err2) == SERVO_OK)
            out("  id %u: model %u%s\r\n", id, get16(m), err ? " (fault flags set)" : "");
        else
            out("  id %u: answers ping, model unreadable\r\n", id);
    }
    out(found ? "%u found\r\n" : "none found (check wiring, 12 V, pull-up)\r\n", found);
}

static void cmd_status(void)
{
    uint8_t a[48], b[15];
    if (!read_bytes(0, sizeof a, a) || !read_bytes(REG_PRESENT_POS, sizeof b, b))
        return;

    int32_t pos = signed_mag(get16(&b[0]), 15);
    int32_t speed = signed_mag(get16(&b[2]), 15);
    int32_t load = signed_mag(get16(&b[4]), 10);
    int32_t current = signed_mag(get16(&b[13]), 15);
    uint16_t goal = get16(&a[REG_GOAL_POS]);
    static const char *const modes[4] = {"position", "wheel", "PWM", "step"};
    uint8_t mode = a[REG_MODE];

    out("servo %u, model %u\r\n", servoId, get16(&a[REG_MODEL]));
    out("  position  %ld", (long)pos);
    out_tenths(" (", steps_to_decideg(pos), " deg)");
    out(", goal %u", goal);
    out_tenths(" (", steps_to_decideg(goal), " deg)");
    out(", speed %ld steps/s, %s\r\n", (long)speed, b[10] ? "moving" : "not moving");
    out_tenths("  supply    ", b[6], " V");
    out(", %u C", b[7]);
    out_tenths(", load ", load, " %");
    out(", current %ld mA\r\n", (long)(current * 65 / 10));
    out("  torque    %s, mode %u (%s), limits %u to %u, offset %ld, fault flags 0x%02X\r\n",
        a[REG_TORQUE] ? "on" : "off", mode, mode < 4u ? modes[mode] : "?",
        get16(&a[REG_MIN_ANGLE]), get16(&a[REG_MAX_ANGLE]),
        (long)signed_mag(get16(&a[REG_OFFSET]), 11), b[9]);
}

/* ---- Parsing ---- */

static char lower(char c)
{
    return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

static bool word_is(const char *s, size_t len, const char *word)
{
    size_t i = 0;
    for (; i < len && word[i]; i++)
        if (lower(s[i]) != word[i])
            return false;
    return i == len && word[i] == '\0';
}

static bool next_word(const char **s, size_t *len, const char **word, size_t *wordLen)
{
    while (*len && (**s == ' ' || **s == '\t')) {
        (*s)++;
        (*len)--;
    }
    if (*len == 0)
        return false;
    *word = *s;
    while (*len && **s != ' ' && **s != '\t') {
        (*s)++;
        (*len)--;
    }
    *wordLen = (size_t)(*s - *word);
    return true;
}

/* Decimal, or hex with 0x. */
static bool parse_uint(const char *s, size_t len, uint32_t max, uint32_t *value)
{
    uint32_t base = 10, v = 0;
    if (len > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        base = 16;
        s += 2;
        len -= 2;
    }
    if (len == 0 || len > 8)
        return false;
    for (size_t i = 0; i < len; i++) {
        char c = lower(s[i]);
        uint32_t d;
        if (c >= '0' && c <= '9')
            d = (uint32_t)(c - '0');
        else if (base == 16 && c >= 'a' && c <= 'f')
            d = (uint32_t)(c - 'a' + 10);
        else
            return false;
        v = v * base + d;
    }
    if (v > max)
        return false;
    *value = v;
    return true;
}

/* Degrees with up to 2 decimals, in hundredths. */
static bool parse_hundredths(const char *s, size_t len, uint32_t *value)
{
    uint32_t whole = 0, frac = 0;
    int fracDigits = 0;
    bool dot = false, digits = false;
    for (size_t i = 0; i < len; i++) {
        if (s[i] == '.' && !dot) {
            dot = true;
            continue;
        }
        if (s[i] < '0' || s[i] > '9')
            return false;
        digits = true;
        if (!dot) {
            whole = whole * 10u + (uint32_t)(s[i] - '0');
            if (whole > 100000u)
                return false;
        } else if (fracDigits < 2) {
            frac = frac * 10u + (uint32_t)(s[i] - '0');
            fracDigits++;
        }
    }
    if (!digits)
        return false;
    for (; fracDigits < 2; fracDigits++)
        frac *= 10u;
    *value = whole * 100u + frac;
    return true;
}

/* ---- Commands ---- */

static void print_help(void)
{
    vcp_puts("Servo test: ST3215 on PA2, 1 Mbaud. Commands:\r\n"
             "  scan                  find servos (ids 0-253)\r\n");
    out("  id [n]                servo to talk to (now %u)\r\n", servoId);
    vcp_puts("  ping\r\n"
             "  status                position, speed, voltage, temperature, load, current...\r\n"
             "  angle <deg> [speed]   move to 0-360 deg (180 = middle)\r\n"
             "  pos <steps> [speed]   move to 0-4095 (2048 = middle)\r\n");
    out("  speed <steps/s>       for moves (now %lu; 0 = fastest; 4096 steps = 1 turn)\r\n"
        "  acc <n>               for moves, x100 steps/s^2 (now %lu; 0 = fastest)\r\n",
        (unsigned long)moveSpeed, (unsigned long)moveAcc);
    vcp_puts("  torque on | off\r\n"
             "  stop                  hold where it is now (also Esc)\r\n"
             "  read <addr> [len]     raw registers (hex)\r\n"
             "  write <addr> <byte>.. raw registers; EEPROM (below 40): write 55 0 first,\r\n"
             "                        write 55 1 after\r\n"
             "  help\r\n"
             "Numbers can be hex: 0x2A. Up / Down arrows: previous commands.\r\n");
}

static void cmd_move(const char *args, size_t len, bool degrees)
{
    const char *w, *w2, *extra;
    size_t wl, wl2, el;
    uint32_t value, speed = moveSpeed, steps;

    bool ok = next_word(&args, &len, &w, &wl);
    if (ok && degrees) {
        ok = parse_hundredths(w, wl, &value) && value <= 36000u;
        steps = (value * STEPS_PER_TURN + 18000u) / 36000u;
    } else if (ok) {
        ok = parse_uint(w, wl, STEPS_PER_TURN - 1u, &value);
        steps = value;
    }
    if (ok && next_word(&args, &len, &w2, &wl2))
        ok = parse_uint(w2, wl2, 65535u, &speed);
    if (ok && next_word(&args, &len, &extra, &el))
        ok = false;
    if (!ok) {
        out(degrees ? "usage: angle <0-360> [speed]\r\n" : "usage: pos <0-4095> [speed]\r\n");
        return;
    }
    uint32_t saved = moveSpeed;
    moveSpeed = speed;
    move_to(steps);
    moveSpeed = saved;
}

static void cmd_read(const char *args, size_t len)
{
    const char *w;
    size_t wl;
    uint32_t addr, n = 1;
    if (!next_word(&args, &len, &w, &wl) || !parse_uint(w, wl, 255, &addr) ||
        (next_word(&args, &len, &w, &wl) && !parse_uint(w, wl, SERVO_MAX_DATA, &n)) || n == 0) {
        out("usage: read <addr> [len 1-%u]\r\n", SERVO_MAX_DATA);
        return;
    }
    uint8_t data[SERVO_MAX_DATA];
    if (!read_bytes((uint8_t)addr, (uint8_t)n, data))
        return;
    for (uint32_t i = 0; i < n; i++) {
        if (i % 16u == 0)
            out(i ? "\r\n%3lu:" : "%3lu:", (unsigned long)(addr + i));
        out(" %02X", data[i]);
    }
    if (n == 1)
        out("   (%u)", data[0]);
    else if (n == 2)
        out("   (%u)", get16(data));
    out("\r\n");
}

static void cmd_write(const char *args, size_t len)
{
    const char *w;
    size_t wl;
    uint32_t addr, v;
    uint8_t data[SERVO_MAX_DATA];
    uint8_t n = 0;
    bool ok = next_word(&args, &len, &w, &wl) && parse_uint(w, wl, 255, &addr);
    while (ok && next_word(&args, &len, &w, &wl)) {
        ok = n < SERVO_MAX_DATA && parse_uint(w, wl, 255, &v);
        data[n++] = (uint8_t)v;
    }
    if (!ok || n == 0) {
        out("usage: write <addr> <byte> [byte...]\r\n");
        return;
    }
    if (write_bytes((uint8_t)addr, data, n))
        out("wrote %u byte%s at %lu\r\n", n, n == 1 ? "" : "s", (unsigned long)addr);
}

static void run_line(const char *s, size_t len)
{
    const char *cmd, *w;
    size_t cmdLen, wl;
    uint32_t v;
    uint8_t err = 0;

    if (!next_word(&s, &len, &cmd, &cmdLen))
        return;

    if (word_is(cmd, cmdLen, "help") || word_is(cmd, cmdLen, "?")) {
        print_help();
    } else if (word_is(cmd, cmdLen, "scan")) {
        cmd_scan();
    } else if (word_is(cmd, cmdLen, "id")) {
        if (next_word(&s, &len, &w, &wl)) {
            if (!parse_uint(w, wl, 253, &v)) {
                out("usage: id <0-253>\r\n");
                return;
            }
            servoId = (uint8_t)v;
        }
        out("talking to servo %u\r\n", servoId);
    } else if (word_is(cmd, cmdLen, "ping")) {
        enum ServoResult r = servo_ping(servoId, 3000, &err);
        if (report(r, err))
            out("servo %u answered\r\n", servoId);
    } else if (word_is(cmd, cmdLen, "status")) {
        cmd_status();
    } else if (word_is(cmd, cmdLen, "angle")) {
        cmd_move(s, len, true);
    } else if (word_is(cmd, cmdLen, "pos")) {
        cmd_move(s, len, false);
    } else if (word_is(cmd, cmdLen, "speed") || word_is(cmd, cmdLen, "acc")) {
        bool isSpeed = word_is(cmd, cmdLen, "speed");
        if (!next_word(&s, &len, &w, &wl) || !parse_uint(w, wl, isSpeed ? 65535u : 254u, &v)) {
            out(isSpeed ? "usage: speed <0-65535 steps/s>\r\n" : "usage: acc <0-254>\r\n");
            return;
        }
        if (isSpeed)
            moveSpeed = v;
        else
            moveAcc = v;
        out("moves: speed %lu steps/s, acc %lu\r\n", (unsigned long)moveSpeed, (unsigned long)moveAcc);
    } else if (word_is(cmd, cmdLen, "torque")) {
        if (!next_word(&s, &len, &w, &wl) || !(word_is(w, wl, "on") || word_is(w, wl, "off"))) {
            out("usage: torque on | off\r\n");
            return;
        }
        if (word_is(w, wl, "on")) {
            hold_here();                /* so it doesn't jump to an old goal */
            if (write_byte(REG_TORQUE, 1))
                out("torque on\r\n");
        } else if (write_byte(REG_TORQUE, 0)) {
            out("torque off (free to turn by hand)\r\n");
        }
    } else if (word_is(cmd, cmdLen, "stop")) {
        hold_here();
    } else if (word_is(cmd, cmdLen, "read")) {
        cmd_read(s, len);
    } else if (word_is(cmd, cmdLen, "write")) {
        cmd_write(s, len);
    } else {
        out("unknown command \"%.*s\", type help\r\n", (int)cmdLen, cmd);
    }
}

/* ---- Line input ---- */

static char line[LINE_MAX];
static size_t lineLen;
static bool lineOverflow;

static char history[HISTORY_SIZE][LINE_MAX];
static size_t historyLen[HISTORY_SIZE];
static uint32_t historyTotal;
static int browse = -1;                 /* -1: new line; 0: newest shown... */
static char draft[LINE_MAX];
static size_t draftLen;

static void prompt(void)
{
    vcp_puts("> ");
}

static uint32_t history_count(void)
{
    return historyTotal < HISTORY_SIZE ? historyTotal : HISTORY_SIZE;
}

static uint32_t history_slot(uint32_t back)
{
    return (historyTotal - 1u - back) % HISTORY_SIZE;
}

static void history_add(const char *s, size_t len)
{
    size_t i = 0;
    while (i < len && (s[i] == ' ' || s[i] == '\t'))
        i++;
    if (i == len)
        return;
    if (historyTotal > 0) {
        uint32_t newest = history_slot(0);
        if (historyLen[newest] == len && memcmp(history[newest], s, len) == 0)
            return;
    }
    uint32_t slot = historyTotal % HISTORY_SIZE;
    memcpy(history[slot], s, len);
    historyLen[slot] = len;
    historyTotal++;
}

static void line_replace(const char *s, size_t len)
{
    memcpy(line, s, len);
    lineLen = len;
    lineOverflow = false;
    vcp_puts("\r\x1b[K");
    prompt();
    vcp_write(line, lineLen);
}

static void history_up(void)
{
    if ((uint32_t)(browse + 1) >= history_count())
        return;
    if (browse < 0) {
        memcpy(draft, line, lineLen);
        draftLen = lineLen;
    }
    browse++;
    uint32_t slot = history_slot((uint32_t)browse);
    line_replace(history[slot], historyLen[slot]);
}

static void history_down(void)
{
    if (browse < 0)
        return;
    browse--;
    if (browse < 0) {
        line_replace(draft, draftLen);
    } else {
        uint32_t slot = history_slot((uint32_t)browse);
        line_replace(history[slot], historyLen[slot]);
    }
}

enum EscState { ESC_NONE, ESC_START, ESC_SEQUENCE };
static enum EscState esc = ESC_NONE;
static uint32_t escAt;
static bool escParams;

/* The Esc key on its own: stop the servo where it is. */
static void esc_key(void)
{
    vcp_puts("\r\n");
    hold_here();
    prompt();
    vcp_write(line, lineLen);
}

static void handle_byte(uint8_t c)
{
    static uint8_t prev;

    if (esc == ESC_START) {
        if (c == '[' || c == 'O') {
            esc = ESC_SEQUENCE;
            escParams = false;
            return;
        }
        esc = ESC_NONE;
        esc_key();                      /* Esc then something else: it was the Esc key */
    } else if (esc == ESC_SEQUENCE) {
        if (c >= 0x20 && c <= 0x3F) {
            escParams = true;
            return;
        }
        esc = ESC_NONE;
        if (c >= 0x40 && c <= 0x7E) {
            if (!escParams && c == 'A')
                history_up();
            else if (!escParams && c == 'B')
                history_down();
            return;
        }
    }

    if (c == 0x1B) {
        esc = ESC_START;
        escAt = HAL_GetTick();
    } else if (c == 0x03) {             /* Ctrl-C: drop the line */
        vcp_puts("^C\r\n");
        lineLen = 0;
        lineOverflow = false;
        browse = -1;
        prompt();
    } else if (c == '\r' || c == '\n') {
        if (!(c == '\n' && prev == '\r')) {
            vcp_puts("\r\n");
            if (lineOverflow) {
                out("line too long (max %u characters), ignored\r\n", LINE_MAX);
            } else {
                history_add(line, lineLen);
                run_line(line, lineLen);
            }
            lineLen = 0;
            lineOverflow = false;
            browse = -1;
            prompt();
        }
    } else if (c == 0x08 || c == 0x7F) {
        if (lineLen > 0) {
            lineLen--;
            vcp_puts("\b \b");
        }
    } else if (c >= 0x20 && c < 0x7F) {
        if (lineLen < LINE_MAX) {
            line[lineLen++] = (char)c;
            vcp_write((const char *)&c, 1);
        } else {
            lineOverflow = true;
        }
    }
    prev = c;
}

/* ---- Board ---- */

static void led_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;
    GPIOC->BSRR = 1u << LED_PIN;                    /* off */
    GPIOC->MODER = (GPIOC->MODER & ~(3u << (2u * LED_PIN))) | (1u << (2u * LED_PIN));
}

static void led(bool on)
{
    GPIOC->BSRR = on ? 1u << (LED_PIN + 16u) : 1u << LED_PIN;
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    led_init();
    servo_port_init();
    MX_USB_DEVICE_Init();

    for (;;) {
        vcp_poll();
        led(HAL_GetTick() % 1000u < 50u);

        if (vcp_just_opened()) {
            vcp_puts("\r\n");
            print_help();
            prompt();
            vcp_write(line, lineLen);
        }

        int c = vcp_getc();
        if (c >= 0)
            handle_byte((uint8_t)c);
        else if (esc == ESC_START && HAL_GetTick() - escAt >= ESC_WAIT_MS) {
            esc = ESC_NONE;
            esc_key();
        }
    }
}

/* Heap for newlib's vsnprintf: from the end of .bss up to the stack. */
void *_sbrk(ptrdiff_t incr)
{
    extern uint8_t _end, _estack, _Min_Stack_Size;
    static uint8_t *heapEnd = &_end;
    uint8_t *limit = &_estack - (uint32_t)&_Min_Stack_Size;
    if (heapEnd + incr > limit)
        return (void *)-1;
    uint8_t *prev = heapEnd;
    heapEnd += incr;
    return prev;
}

/* Called by HAL_Init(). */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

/* HSE 25 MHz -> 96 MHz core, 48 MHz USB. APB1 48 MHz, APB2 96 MHz. */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 25;
    osc.PLL.PLLN = 192;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
        Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK)
        Error_Handler();
}

/* Fatal error: flash the LED fast forever. */
void Error_Handler(void)
{
    __disable_irq();
    led_init();
    for (;;) {
        GPIOC->ODR ^= 1u << LED_PIN;
        for (volatile uint32_t i = 0; i < 100000u; i++) {
        }
    }
}

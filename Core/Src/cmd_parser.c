#include "cmd_parser.h"
#include "led_pwm.h"
#include "i2c_bridge.h"
#include "usbd_cdc_if.h"
#include "main.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#define RX_RING_SIZE   512
#define LINE_BUF_SIZE  256
#define TX_BUF_SIZE    512
#define MAX_ARGS       16
#define TX_TIMEOUT_MS  50

/* ---- RX ring (producer = USB IRQ, consumer = main loop) ---- */
static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

/* ---- Line accumulator (main-loop only) ---- */
static char     line_buf[LINE_BUF_SIZE];
static uint16_t line_len;
static bool     line_overflow;

/* ---- TX scratch ---- */
static char tx_buf[TX_BUF_SIZE];

/* =========================================================== */
/* RX                                                           */
/* =========================================================== */

void cmdp_push_bytes(const uint8_t *data, uint32_t len)
{
    for (uint32_t i = 0; i < len; ++i) {
        uint16_t next = (uint16_t)((rx_head + 1u) % RX_RING_SIZE);
        if (next == rx_tail) return;          /* full, drop rest */
        rx_ring[rx_head] = data[i];
        rx_head = next;
    }
}

static int rx_pop(void)
{
    if (rx_head == rx_tail) return -1;
    int b = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) % RX_RING_SIZE);
    return b;
}

/* =========================================================== */
/* TX                                                           */
/* =========================================================== */

static void send_line(const char *s)
{
    uint16_t n = (uint16_t)strlen(s);
    uint32_t deadline = HAL_GetTick() + TX_TIMEOUT_MS;
    while (HAL_GetTick() < deadline) {
        if (CDC_Transmit_FS((uint8_t *)s, n) == USBD_OK) return;
    }
    /* dropped — host not reading or USB disconnected */
}

static void resp_ok(void)                  { send_line("OK\n"); }
static void resp_err(const char *reason)
{
    snprintf(tx_buf, sizeof(tx_buf), "ERR_%s\n", reason);
    send_line(tx_buf);
}

/* =========================================================== */
/* Helpers                                                      */
/* =========================================================== */

/* Accepts: 123 (decimal), 0xFF / 0XFF (hex), 0b1010 / 0B1010 (binary).
 * Returns false on empty string, trailing garbage, or overflow. */
static bool parse_uint(const char *s, uint32_t *out)
{
    if (!s || !*s) return false;
    char *end;
    unsigned long v;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        if (!s[2]) return false;
        v = strtoul(s + 2, &end, 16);
    } else if (s[0] == '0' && (s[1] == 'b' || s[1] == 'B')) {
        if (!s[2]) return false;
        v = strtoul(s + 2, &end, 2);
    } else {
        v = strtoul(s, &end, 10);
    }
    if (end == s || *end != '\0') return false;
    *out = (uint32_t)v;
    return true;
}

static void str_upper(char *s)
{
    for (; *s; ++s) *s = (char)toupper((unsigned char)*s);
}

static void str_trim(char *s)
{
    char *p = s;
    while (*p && isspace((unsigned char)*p)) p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n - 1])) s[--n] = '\0';
}

/* =========================================================== */
/* Handlers                                                     */
/* =========================================================== */

static void cmd_led_set(int argc, char **argv)
{
    if (argc != 3) { resp_err("ARGS"); return; }
    uint32_t r, g, b;
    if (!parse_uint(argv[0], &r) ||
        !parse_uint(argv[1], &g) ||
        !parse_uint(argv[2], &b)) {
        resp_err("SYNTAX"); return;
    }
    if (r > 255 || g > 255 || b > 255) { resp_err("RANGE"); return; }
    led_set((uint8_t)r, (uint8_t)g, (uint8_t)b);
    resp_ok();
}

static void cmd_led_get(int argc, char **argv)
{
    (void)argc; (void)argv;
    uint8_t r, g, b;
    led_get(&r, &g, &b);
    snprintf(tx_buf, sizeof(tx_buf), "OK %u %u %u\n", r, g, b);
    send_line(tx_buf);
}

/* ---- I²C handlers ---- */

static void cmd_i2c_scan(int argc, char **argv)
{
    (void)argc; (void)argv;
    uint8_t addrs[127];
    int n = i2cb_scan(addrs);
    int off = snprintf(tx_buf, sizeof(tx_buf), "OK");
    for (int i = 0; i < n; ++i) {
        off += snprintf(tx_buf + off, sizeof(tx_buf) - off, " 0x%02X", addrs[i]);
    }
    snprintf(tx_buf + off, sizeof(tx_buf) - off, "\n");
    send_line(tx_buf);
}

static void cmd_i2c_probe(int argc, char **argv)
{
    if (argc != 1) { resp_err("ARGS"); return; }
    uint32_t addr;
    if (!parse_uint(argv[0], &addr)) { resp_err("SYNTAX"); return; }
    if (addr == 0 || addr > 0x7F)    { resp_err("RANGE");  return; }
    if (i2cb_probe((uint8_t)addr))   resp_ok();
    else                              resp_err("NACK");
}

/* Internal: emit "OK XX XX ..." for a buffer of bytes. */
static void emit_bytes(const uint8_t *buf, uint16_t len)
{
    int off = snprintf(tx_buf, sizeof(tx_buf), "OK");
    for (uint16_t i = 0; i < len; ++i) {
        off += snprintf(tx_buf + off, sizeof(tx_buf) - off, " %02X", buf[i]);
    }
    snprintf(tx_buf + off, sizeof(tx_buf) - off, "\n");
    send_line(tx_buf);
}

static void cmd_i2c_read(int argc, char **argv)
{
    if (argc != 3) { resp_err("ARGS"); return; }
    uint32_t addr, reg, len;
    if (!parse_uint(argv[0], &addr) ||
        !parse_uint(argv[1], &reg)  ||
        !parse_uint(argv[2], &len)) { resp_err("SYNTAX"); return; }
    if (addr == 0 || addr > 0x7F || reg > 0xFF || len == 0 || len > 128) {
        resp_err("RANGE"); return;
    }
    uint8_t buf[128];
    i2cb_status_t st = i2cb_read((uint8_t)addr, (uint8_t)reg, buf, (uint16_t)len);
    if (st != I2CB_OK) { resp_err(i2cb_status_str(st)); return; }
    emit_bytes(buf, (uint16_t)len);
}

static void cmd_i2c_write(int argc, char **argv)
{
    if (argc < 3) { resp_err("ARGS"); return; }
    uint32_t addr, reg;
    if (!parse_uint(argv[0], &addr) ||
        !parse_uint(argv[1], &reg)) { resp_err("SYNTAX"); return; }
    if (addr == 0 || addr > 0x7F || reg > 0xFF) { resp_err("RANGE"); return; }
    int n = argc - 2;
    if (n > 128) { resp_err("RANGE"); return; }
    uint8_t buf[128];
    for (int i = 0; i < n; ++i) {
        uint32_t v;
        if (!parse_uint(argv[2 + i], &v)) { resp_err("SYNTAX"); return; }
        if (v > 0xFF) { resp_err("RANGE"); return; }
        buf[i] = (uint8_t)v;
    }
    i2cb_status_t st = i2cb_write((uint8_t)addr, (uint8_t)reg, buf, (uint16_t)n);
    if (st != I2CB_OK) { resp_err(i2cb_status_str(st)); return; }
    resp_ok();
}

static void cmd_i2c_writeraw(int argc, char **argv)
{
    if (argc < 2) { resp_err("ARGS"); return; }
    uint32_t addr;
    if (!parse_uint(argv[0], &addr)) { resp_err("SYNTAX"); return; }
    if (addr == 0 || addr > 0x7F)    { resp_err("RANGE");  return; }
    int n = argc - 1;
    if (n > 128) { resp_err("RANGE"); return; }
    uint8_t buf[128];
    for (int i = 0; i < n; ++i) {
        uint32_t v;
        if (!parse_uint(argv[1 + i], &v)) { resp_err("SYNTAX"); return; }
        if (v > 0xFF) { resp_err("RANGE"); return; }
        buf[i] = (uint8_t)v;
    }
    i2cb_status_t st = i2cb_writeraw((uint8_t)addr, buf, (uint16_t)n);
    if (st != I2CB_OK) { resp_err(i2cb_status_str(st)); return; }
    resp_ok();
}

static void cmd_i2c_readraw(int argc, char **argv)
{
    if (argc != 2) { resp_err("ARGS"); return; }
    uint32_t addr, len;
    if (!parse_uint(argv[0], &addr) ||
        !parse_uint(argv[1], &len)) { resp_err("SYNTAX"); return; }
    if (addr == 0 || addr > 0x7F || len == 0 || len > 128) {
        resp_err("RANGE"); return;
    }
    uint8_t buf[128];
    i2cb_status_t st = i2cb_readraw((uint8_t)addr, buf, (uint16_t)len);
    if (st != I2CB_OK) { resp_err(i2cb_status_str(st)); return; }
    emit_bytes(buf, (uint16_t)len);
}

static void cmd_i2c_reset(int argc, char **argv)
{
    (void)argc; (void)argv;
    i2cb_reset();
    resp_ok();
}

static void cmd_i2c_setspeed(int argc, char **argv)
{
    if (argc != 1) { resp_err("ARGS"); return; }
    uint32_t hz;
    if (!parse_uint(argv[0], &hz)) { resp_err("SYNTAX"); return; }
    if (!i2cb_setspeed(hz))        { resp_err("RANGE");  return; }
    resp_ok();
}

/* =========================================================== */
/* Dispatch                                                     */
/* =========================================================== */

typedef struct {
    const char *name;
    void (*handler)(int argc, char **argv);
} cmd_entry_t;

static const cmd_entry_t cmd_table[] = {
    { "LED.SETCOLOR",  cmd_led_set },
    { "LED.GETCOLOR?", cmd_led_get },
    { "I2C.SCAN",      cmd_i2c_scan },
    { "I2C.PROBE",     cmd_i2c_probe },
    { "I2C.READ",      cmd_i2c_read },
    { "I2C.WRITE",     cmd_i2c_write },
    { "I2C.WRITERAW",  cmd_i2c_writeraw },
    { "I2C.READRAW",   cmd_i2c_readraw },
    { "I2C.RESET",     cmd_i2c_reset },
    { "I2C.SETSPEED",  cmd_i2c_setspeed },
};
static void dispatch_line(char *line)
{
    str_trim(line);
    if (line[0] == '\0') return;

    /* Split "CMD(a,b,c)" into name + args. Bare "CMD" or "CMD?" is allowed too. */
    char *args_start = NULL;
    char *paren = strchr(line, '(');
    if (paren) {
        char *close = strrchr(line, ')');
        if (!close || close < paren) { resp_err("SYNTAX"); return; }
        *paren = '\0';
        *close = '\0';
        args_start = paren + 1;
    }

    str_trim(line);
    str_upper(line);

    /* Tokenize args on commas, trim each. */
    char *argv[MAX_ARGS];
    int   argc = 0;
    if (args_start) {
        char *p = args_start;
        while (*p) {
            while (*p && isspace((unsigned char)*p)) p++;
            if (!*p) break;
            if (argc >= MAX_ARGS) { resp_err("ARGS"); return; }
            argv[argc++] = p;
            while (*p && *p != ',') p++;
            if (*p == ',') { *p = '\0'; p++; }
        }
        for (int i = 0; i < argc; ++i) {
            char *e = argv[i] + strlen(argv[i]);
            while (e > argv[i] && isspace((unsigned char)e[-1])) *--e = '\0';
        }
    }

    for (size_t i = 0; i < sizeof(cmd_table)/sizeof(cmd_table[0]); ++i) {
        if (strcmp(line, cmd_table[i].name) == 0) {
            cmd_table[i].handler(argc, argv);
            return;
        }
    }
    resp_err("UNKNOWN_CMD");
}

/* =========================================================== */
/* Init + main task                                             */
/* =========================================================== */

void cmdp_init(void)
{
    rx_head = rx_tail = 0;
    line_len = 0;
    line_overflow = false;
}

void cmdp_task(void)
{
    int b;
    while ((b = rx_pop()) >= 0) {
        char c = (char)b;
        if (c == '\n' || c == '\r') {
            if (line_overflow) {
                resp_err("OVERFLOW");
                line_overflow = false;
                line_len = 0;
            } else if (line_len > 0) {
                line_buf[line_len] = '\0';
                dispatch_line(line_buf);
                line_len = 0;
            }
        } else if (line_len < LINE_BUF_SIZE - 1) {
            line_buf[line_len++] = c;
        } else {
            line_overflow = true;
        }
    }
}

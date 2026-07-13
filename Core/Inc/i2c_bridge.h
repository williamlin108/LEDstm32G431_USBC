#ifndef I2C_BRIDGE_H
#define I2C_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    I2CB_OK = 0,
    I2CB_NACK,      /* slave did not ACK (wrong addr, busy EEPROM, etc) */
    I2CB_TIMEOUT,   /* clock-stretch or missing pull-ups */
    I2CB_BUS,       /* arbitration lost / bus error / other */
} i2cb_status_t;

const char *i2cb_status_str(i2cb_status_t st);

/* Scan addresses 0x01..0x7F. Writes responders to out_addrs (caller provides
 * room for up to 127 bytes). Returns count. */
int  i2cb_scan(uint8_t *out_addrs);

/* Single-address ACK test. */
bool i2cb_probe(uint8_t addr);

/* Register-pointer transactions: writes reg byte, then reads/writes data. */
i2cb_status_t i2cb_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
i2cb_status_t i2cb_write(uint8_t addr, uint8_t reg, const uint8_t *buf, uint16_t len);

/* Raw transactions: no implicit register-pointer byte. */
i2cb_status_t i2cb_writeraw(uint8_t addr, const uint8_t *buf, uint16_t len);
i2cb_status_t i2cb_readraw(uint8_t addr, uint8_t *buf, uint16_t len);

/* DeInit/Init peripheral to recover from a hang. */
void i2cb_reset(void);

/* Reconfigure speed. Accepts 100000, 400000, 1000000 (Hz). */
bool i2cb_setspeed(uint32_t hz);
uint32_t i2cb_get_speed(void);

#ifdef __cplusplus
}
#endif
#endif

#include "i2c_bridge.h"
#include "main.h"

#define I2C_TIMEOUT_MS  10
#define I2C_PROBE_TRIES 1

extern I2C_HandleTypeDef hi2c2;

static uint32_t cur_speed_hz = 100000;

/* CubeMX-style timing words for PCLK = 170 MHz, analog filter ON, digital 0.
 * 100 kHz value is the verified one from your .ioc; 400k/1M are nominal —
 * tighten if you put a scope on it. */
#define TIMING_100K   0x40B285C2u
#define TIMING_400K   0x10D0AAFFu
#define TIMING_1M     0x00802172u

const char *i2cb_status_str(i2cb_status_t st)
{
    switch (st) {
        case I2CB_OK:      return "OK";
        case I2CB_NACK:    return "I2C_NACK";
        case I2CB_TIMEOUT: return "I2C_TIMEOUT";
        case I2CB_BUS:     return "I2C_BUS";
        default:           return "I2C_UNKNOWN";
    }
}

static i2cb_status_t map_err(HAL_StatusTypeDef hs)
{
    if (hs == HAL_OK)      return I2CB_OK;
    if (hs == HAL_TIMEOUT) return I2CB_TIMEOUT;
    uint32_t e = HAL_I2C_GetError(&hi2c2);
    if (e & HAL_I2C_ERROR_AF)                       return I2CB_NACK;
    if (e & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO)) return I2CB_BUS;
    return I2CB_BUS;
}

static void reinit(void)
{
    HAL_I2C_DeInit(&hi2c2);
    hi2c2.Init.Timing = (cur_speed_hz == 1000000) ? TIMING_1M
                      : (cur_speed_hz == 400000)  ? TIMING_400K
                      :                              TIMING_100K;
    HAL_I2C_Init(&hi2c2);
    HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0);
}

static i2cb_status_t finish(HAL_StatusTypeDef hs)
{
    if (hs == HAL_OK) return I2CB_OK;
    i2cb_status_t st = map_err(hs);
    reinit();   /* clear any latched bus state */
    return st;
}

int i2cb_scan(uint8_t *out)
{
    int n = 0;
    for (uint8_t a = 0x01; a < 0x80; ++a) {
        if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(a << 1),
                                  I2C_PROBE_TRIES, I2C_TIMEOUT_MS) == HAL_OK) {
            out[n++] = a;
        }
    }
    return n;
}

bool i2cb_probe(uint8_t addr)
{
    return HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(addr << 1),
                                 I2C_PROBE_TRIES, I2C_TIMEOUT_MS) == HAL_OK;
}

i2cb_status_t i2cb_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return finish(HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(addr << 1),
                                   reg, I2C_MEMADD_SIZE_8BIT,
                                   buf, len, I2C_TIMEOUT_MS));
}

i2cb_status_t i2cb_write(uint8_t addr, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    return finish(HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(addr << 1),
                                    reg, I2C_MEMADD_SIZE_8BIT,
                                    (uint8_t *)buf, len, I2C_TIMEOUT_MS));
}

i2cb_status_t i2cb_writeraw(uint8_t addr, const uint8_t *buf, uint16_t len)
{
    return finish(HAL_I2C_Master_Transmit(&hi2c2, (uint16_t)(addr << 1),
                                          (uint8_t *)buf, len, I2C_TIMEOUT_MS));
}

i2cb_status_t i2cb_readraw(uint8_t addr, uint8_t *buf, uint16_t len)
{
    return finish(HAL_I2C_Master_Receive(&hi2c2, (uint16_t)(addr << 1),
                                         buf, len, I2C_TIMEOUT_MS));
}

void i2cb_reset(void) { reinit(); }

bool i2cb_setspeed(uint32_t hz)
{
    if (hz != 100000 && hz != 400000 && hz != 1000000) return false;
    cur_speed_hz = hz;
    reinit();
    return true;
}

uint32_t i2cb_get_speed(void) { return cur_speed_hz; }

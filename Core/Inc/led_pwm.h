#ifndef LED_PWM_H
#define LED_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void led_init(void);
void led_set(uint8_t r, uint8_t g, uint8_t b);
void led_get(uint8_t *r, uint8_t *g, uint8_t *b);

#ifdef __cplusplus
}
#endif
#endif

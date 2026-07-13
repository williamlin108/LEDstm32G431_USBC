#include "led_pwm.h"
#include "main.h"

/*
 * RGB LED: common anode to 3.3V, drivers active-low.
 * Timer in PWM Mode 1: channel is ACTIVE while CNT < CCR.
 * Active output = HIGH (with default polarity) = LED OFF.
 *
 *   CCR = 0        -> CNT < 0 never true -> output always LOW  -> LED full ON
 *   CCR = ARR+1    -> CNT < CCR  always  -> output always HIGH -> LED true OFF
 *                                            (no edge, no 1-tick glow)
 *
 * Channel map:
 *   Red   -> PB7 -> TIM4_CH2
 *   Green -> PB6 -> TIM4_CH1
 *   Blue  -> PB5 -> TIM3_CH2
 */

#define LED_ARR  999u
#define LED_OFF  (LED_ARR + 1u)

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

static uint8_t cur_r, cur_g, cur_b;

static uint32_t brightness_to_ccr(uint8_t br)
{
    if (br == 0)   return LED_OFF;   /* true off */
    if (br == 255) return 0;          /* true full on */
    return ((uint32_t)(255u - br) * (LED_ARR + 1u) + 127u) / 255u;
}

void led_init(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, LED_OFF); /* R */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, LED_OFF); /* G */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, LED_OFF); /* B */

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    cur_r = cur_g = cur_b = 0;
}

void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, brightness_to_ccr(r));
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, brightness_to_ccr(g));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, brightness_to_ccr(b));
    cur_r = r; cur_g = g; cur_b = b;
}

void led_get(uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = cur_r; *g = cur_g; *b = cur_b;
}

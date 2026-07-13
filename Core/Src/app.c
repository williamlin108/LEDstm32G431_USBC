#include "app.h"
#include "led_pwm.h"
#include "cmd_parser.h"

void app_init(void)
{
    led_init();
    cmdp_init();
}

void app_task(void)
{
    cmdp_task();
}

#include "bsp_led.h"

void use_led_ON(void)
{
    DL_GPIO_setPins(use_led_PORT,use_led_PIN_22_PIN);
}

void use_led_OFF(void)
{
    DL_GPIO_clearPins(use_led_PORT,use_led_PIN_22_PIN);
}

void use_led_TOGGLE(void)
{
    DL_GPIO_togglePins(use_led_PORT,use_led_PIN_22_PIN);
}

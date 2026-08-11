#include "periph.h"
#include "driver/gpio.h"

#define PERIPH_LED_GPIO GPIO_NUM_2

void periph_init(void)
{
    gpio_reset_pin(PERIPH_LED_GPIO);
    gpio_set_direction(PERIPH_LED_GPIO, GPIO_MODE_OUTPUT);
}

void periph_toggle_led(bool state)
{
    gpio_set_level(PERIPH_LED_GPIO, state);
}



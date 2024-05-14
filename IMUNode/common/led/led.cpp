#include "led.h"

void Led::init(gpio_num_t led_gpio_pin) {
  led_pin = led_gpio_pin;
  gpio_set_direction(led_pin, GPIO_MODE_OUTPUT);
}

void Led::on() {
  led_state = 1;
  gpio_set_level(led_pin, led_state);
}

void Led::off() {
  led_state = 0;
  gpio_set_level(led_pin, led_state);
}

void Led::toggle() {
  led_state = !led_state;
  gpio_set_level(led_pin, led_state);
}


#ifndef  LED_COMP_H
#define  LED_COMP_H

#include <driver/gpio.h>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

class Led {
public:
    void init(gpio_num_t led_gpio_pin);
    void on();
    void off();
    void toggle();

private:
    uint8_t led_state = 0;
    gpio_num_t led_pin;
};

class BuiltInLed : public Led {
public:
    void init() {
      constexpr gpio_num_t BULTIN_LED_PIN = GPIO_NUM_2;
      Led::init(BULTIN_LED_PIN);
    }
};

inline BuiltInLed builtin_led;

#ifdef __cplusplus
};
#endif

#endif // LED_COMP_H

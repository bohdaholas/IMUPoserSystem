#ifndef BUTTON_COMP_H
#define BUTTON_COMP_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <functional>

#ifdef __cplusplus
extern "C" {
#endif

class Button {
public:
    void init(gpio_num_t btn_pin);
    static void IRAM_ATTR button_isr_handler(void* arg);
    void conf_btn_press_detection();
    void set_cb_on_btn_press(std::function<void()> cb);
    void run_cb_on_btn_press();
    void wait_for_btn_press();

    SemaphoreHandle_t debounce_semaphore = nullptr;
private:
    gpio_num_t btn_pin = GPIO_NUM_NC;
    std::function<void()> callback;
};

class BuiltInButton : public Button {
public:
    void init() {
      constexpr gpio_num_t BULTIN_BTN_PIN = GPIO_NUM_0;
      Button::init(BULTIN_BTN_PIN);
    }
};

inline BuiltInButton builtin_button;

#ifdef __cplusplus
};
#endif

#endif //BUTTON_COMP_H

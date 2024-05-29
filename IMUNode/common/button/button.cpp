#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_attr.h>
#include "button.h"

void IRAM_ATTR Button::button_isr_handler(void *arg) {
  auto btn_ptr = reinterpret_cast<Button *>(arg);
  xSemaphoreGiveFromISR(btn_ptr->debounce_semaphore, nullptr);
}

void Button::init(gpio_num_t button_pin) {
  btn_pin = button_pin;
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << btn_pin);
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io_conf);
}

void Button::conf_btn_press_detection() {
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
  gpio_isr_handler_add(btn_pin, button_isr_handler, this);

  debounce_semaphore = xSemaphoreCreateBinary();
}

void Button::set_cb_on_btn_press(std::function<void()> cb) {
  callback = std::move(cb);
}

void Button::run_cb_on_btn_press() {
  constexpr size_t DEBOUNCE_DELAY_MS = 200;
  if (xSemaphoreTake(debounce_semaphore, portMAX_DELAY) == pdTRUE) {
    callback();
    vTaskDelay(DEBOUNCE_DELAY_MS);
  }
}

void Button::wait_for_btn_press() {
  constexpr size_t DEBOUNCE_DELAY_MS = 200;
  if (xSemaphoreTake(debounce_semaphore, portMAX_DELAY) == pdTRUE) {
    vTaskDelay(DEBOUNCE_DELAY_MS);
  }
}



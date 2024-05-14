#include <esp_log.h>
#include <driver/gpio.h>
#include <sstream>
#include "common_constants.h"
#include "mqtt.h"
#include "wifi.h"

static const char *TAG = "MQTT";
SemaphoreHandle_t debounce_semaphore;
#define LED_PIN GPIO_NUM_2
#define BUTTON_PIN GPIO_NUM_0

static void IRAM_ATTR button_isr_handler(void* arg) {
  xSemaphoreGiveFromISR(debounce_semaphore, nullptr);
}

void configure_button_n_led() {
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io_conf);

  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
  gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, nullptr);

  debounce_semaphore = xSemaphoreCreateBinary();
}


void MQTT::init() {
  std::stringstream uri_ss;
  uri_ss << "mqtt://" << HOST_IP_ADDRESS << ":" << std::to_string(MQTT_PORT);
  std::string uri = uri_ss.str();
  const esp_mqtt_client_config_t mqtt_cfg = {
      .broker {
          .address {
              .uri = uri.c_str()
          }
      }
  };
  vTaskDelay(pdMS_TO_TICKS(500));
  configure_button_n_led();
  wifi.wait_till_connected();
  xTaskCreate(pub_msg_on_btn_press, "pub_msg_on_btn_press", 2048, nullptr, 10, nullptr);
  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  if (mqtt_client == nullptr) {
    ESP_LOGE(TAG, "Failed to initialize MQTT client");
    return;
  }
  esp_mqtt_client_register_event(mqtt_client, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID), mqtt_event_handler, nullptr);
  esp_mqtt_client_start(mqtt_client);
}

void MQTT::mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  auto event = (esp_mqtt_event_handle_t) event_data;
  switch (event->event_id) {
    case MQTT_EVENT_CONNECTED:
      ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
      break;
    case MQTT_EVENT_DISCONNECTED:
      ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
      wifi.wait_till_connected();
      break;
    case MQTT_EVENT_SUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");
      break;
    case MQTT_EVENT_UNSUBSCRIBED:
      ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED");
      break;
    case MQTT_EVENT_PUBLISHED:
      ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED");
      break;
    case MQTT_EVENT_DATA:
      ESP_LOGI(TAG, "MQTT_EVENT_DATA");
      break;
    case MQTT_EVENT_ERROR:
      ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
      break;
    case MQTT_EVENT_ANY:
      ESP_LOGI(TAG, "MQTT_EVENT_ANY");
      break;
    case MQTT_EVENT_BEFORE_CONNECT:
      ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
      break;
    case MQTT_EVENT_DELETED:
      ESP_LOGI(TAG, "MQTT_EVENT_DELETED");
      break;
    case MQTT_USER_EVENT:
      ESP_LOGI(TAG, "MQTT_USER_EVENT");
      break;
  }
}

void MQTT::pub_msg_on_btn_press(void *pvParameters) {
  constexpr const char* topic = "tpose";
  int led_state = 0;
  while (1) {
    if (xSemaphoreTake(debounce_semaphore, portMAX_DELAY) == pdTRUE) {
      printf("Button pressed, sending MQTT message.\n");
      led_state = !led_state;
      gpio_set_level(LED_PIN, led_state);
      wifi.wait_till_connected();
      esp_mqtt_client_publish(mqtt_manager.mqtt_client, topic, "", 0, 1, 0);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}
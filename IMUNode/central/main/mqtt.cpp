#include <esp_log.h>
#include <driver/gpio.h>
#include <sstream>
#include "led.h"
#include "button.h"
#include "common_constants.h"
#include "mqtt.h"
#include "wifi.h"

static const char *TAG = "MQTT";

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

void MQTT::send_tpose_req_cb() {
  constexpr const char* topic = "tpose";
  printf("Button pressed, sending MQTT message.\n");
  builtin_led.toggle();
  wifi.wait_till_connected();
  esp_mqtt_client_publish(mqtt_manager.mqtt_client, topic, "", 0, 1, 0);
}

void MQTT::pub_msg_on_btn_press(void *pvParameters) {
  builtin_button.set_cb_on_btn_press(send_tpose_req_cb);
  while (true) {
    builtin_button.run_cb_on_btn_press();
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}
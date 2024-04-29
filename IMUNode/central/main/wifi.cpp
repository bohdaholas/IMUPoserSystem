#include <esp_wifi.h>
#include <cstring>
#include <esp_log.h>
#include "wifi.h"

constexpr const char *SSID = "s23ultra";
constexpr const char *PASSWORD = "12341234";

void Wifi::init() {
  is_connected_semaphore = xSemaphoreCreateBinary();
  connect_to_wifi();
}

void Wifi::wait_till_connected() {
  if (xSemaphoreTake(wifi.is_connected_semaphore, portMAX_DELAY) != pdTRUE) {
    ESP_LOGE("wifi", "Error occurred when taking semaphore");
  }
  xSemaphoreGive(wifi.is_connected_semaphore);
}

void Wifi::connect_to_wifi() {
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&wifi_initiation);
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, nullptr);
  esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, nullptr);
  wifi_config_t wifi_configuration;
  memset(&wifi_configuration, 0, sizeof(wifi_configuration));
  strcpy((char*)wifi_configuration.sta.ssid, SSID);
  strcpy((char*)wifi_configuration.sta.password, PASSWORD);
  esp_wifi_set_config(WIFI_IF_STA, &wifi_configuration);
  esp_wifi_start();
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_connect();
}

void Wifi::wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data) {
  if(event_id == WIFI_EVENT_STA_START)
  {
    printf("WIFI CONNECTING....\n");
  }
  else if (event_id == WIFI_EVENT_STA_CONNECTED)
  {
    xSemaphoreGive(wifi.is_connected_semaphore);
    printf("WiFi CONNECTED\n");
  }
  else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    if (xSemaphoreTake(wifi.is_connected_semaphore, portMAX_DELAY) != pdTRUE) {
      ESP_LOGE("wifi", "Error occurred when taking semaphore");
    }
    printf("WiFi lost connection\n");
    esp_wifi_connect();
    printf("Retrying to Connect...\n");
  }
  else if (event_id == IP_EVENT_STA_GOT_IP)
  {
    printf("Wifi got IP...\n\n");
  }
}

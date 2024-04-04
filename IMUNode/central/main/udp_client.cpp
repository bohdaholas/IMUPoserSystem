#include <esp_log.h>
#include "udp_client.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/sockets.h"
#include "gatt.h"

constexpr const char *SSID = "s23ultra";
constexpr const char *PASSWORD = "12341234";
constexpr const char *UDP_IP_ADDRESS = "192.168.72.165";
constexpr size_t UDP_PORT = 1234;

void UdpClient::udp_send_task(void *pvParameters) {
  char buff[100];
  sprintf(buff, "%f,%f,%f\n", gatt.ax, gatt.ay, gatt.az);

  if (sendto(udp_client.udp_socket, buff, strlen(buff), 0,
             (struct sockaddr *)& udp_client.udp_server_addr, sizeof(udp_server_addr)) < 0) {
    ESP_LOGE("udp_send_task", "Error occurred during sending: errno %d", errno);
  } else {
    ESP_LOGI("udp_send_task", "Packet sent successfully");
  }

  vTaskDelete(nullptr);
}

void UdpClient::udp_timer_callback(TimerHandle_t xTimer) {
  xTaskCreate(udp_send_task, "udp_send_task", 2048, nullptr, 5, nullptr);
}

void UdpClient::init() {
  connect_to_wifi();

  udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_socket < 0) {
    ESP_LOGE("udp_socket", "Failed to create socket: errno %d", errno);
    return;
  }

  udp_server_addr.sin_addr.s_addr = inet_addr(UDP_IP_ADDRESS);
  udp_server_addr.sin_family = AF_INET;
  udp_server_addr.sin_port = htons(UDP_PORT);

  udp_timer_handle = xTimerCreate("udp_timer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0, udp_timer_callback);

  if (udp_timer_handle == nullptr) {
    ESP_LOGE("app_main", "Timer create failed");
  } else {
    if (xTimerStart(udp_timer_handle, 10) != pdPASS) {
      ESP_LOGE("app_main", "Timer start failed");
    }
  }
}

void UdpClient::wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,
                                   void *event_data) {
  static int retry_num=0;
  if(event_id == WIFI_EVENT_STA_START)
  {
    printf("WIFI CONNECTING....\n");
  }
  else if (event_id == WIFI_EVENT_STA_CONNECTED)
  {
    printf("WiFi CONNECTED\n");
  }
  else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    printf("WiFi lost connection\n");
    if(retry_num<5){esp_wifi_connect();retry_num++;printf("Retrying to Connect...\n");}
  }
  else if (event_id == IP_EVENT_STA_GOT_IP)
  {
    printf("Wifi got IP...\n\n");
  }
}

void UdpClient::connect_to_wifi() {
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
  printf("wifi_init_softap finished. SSID:%s  password:%s", SSID, PASSWORD);
}

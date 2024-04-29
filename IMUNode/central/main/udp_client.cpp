#include <esp_log.h>
#include "udp_client.h"
#include "esp_event.h"
#include "lwip/sockets.h"
#include "gatt.h"
#include "wifi.h"

constexpr const char *UDP_IP_ADDRESS = "192.168.214.165";
constexpr size_t UDP_PORT = 1234;

static void clear_queue(QueueHandle_t xQueue) {
  BaseType_t xQueueEmpty;
  void *pvItem;

  if (xQueue != nullptr) {
    do {
      xQueueEmpty = xQueueReceive(xQueue, &pvItem, 0);
    } while (xQueueEmpty == pdPASS);
  }
}

void UdpClient::udp_send_task(void *pvParameters) {
  for(;;) {
    node_data_t node_data{};
    char buff[100];

    wifi.wait_till_connected();

    if(xQueueReceive(udp_client.nodeDataQueue, &node_data, portMAX_DELAY) != pdPASS) {
      printf("Error when reading from a queue\n");
      continue;
    }

    auto [w, ax, ay, az] = node_data.orientation_quaternion;
    sprintf(buff, "%s,%f,%f,%f,%f\n", node_data.body_loc_str, w, ax, ay, az);

    if (sendto(udp_client.udp_socket, buff, strlen(buff), 0,
               (struct sockaddr *)& udp_client.udp_server_addr, sizeof(udp_server_addr)) < 0) {
      ESP_LOGE("udp_send_task", "Error occurred during sending %s", node_data.body_loc_str);
//      clear_queue(udp_client.nodeDataQueue);
//      vTaskDelay(pdMS_TO_TICKS(1000));
    } else {
      printf("%s,%f,%f,%f,%f\n", node_data.body_loc_str, w, ax, ay, az);
    }
  }
}

void UdpClient::init() {
  udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (udp_socket < 0) {
    ESP_LOGE("udp_socket", "Failed to create socket: errno %d", errno);
    return;
  }

  udp_server_addr.sin_addr.s_addr = inet_addr(UDP_IP_ADDRESS);
  udp_server_addr.sin_family = AF_INET;
  udp_server_addr.sin_port = htons(UDP_PORT);

  nodeDataQueue = xQueueCreate(10, sizeof(node_data_t));
  if (nodeDataQueue == nullptr) {
    printf("Failed to create queue instance\n");
    return;
  }

  xTaskCreate(udp_send_task, "udp_send_task", 4096, nullptr, 1, nullptr);
}


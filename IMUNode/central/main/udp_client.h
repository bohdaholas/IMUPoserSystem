#ifndef CENTRAL_UDP_CLIENT_H
#define CENTRAL_UDP_CLIENT_H

#include <lwip/sockets.h>
#include <esp_event.h>

#ifdef __cplusplus
extern "C" {
#endif

class UdpClient {
public:
    void init();
    static void udp_send_task(void *pvParameters);

    int udp_socket = -1;
    struct sockaddr_in udp_server_addr{};
    QueueHandle_t nodeDataQueue = nullptr;
};

inline UdpClient udp_client;

#ifdef __cplusplus
}
#endif

#endif //CENTRAL_UDP_CLIENT_H

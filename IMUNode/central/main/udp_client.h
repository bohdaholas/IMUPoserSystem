#ifndef CENTRAL_UDP_CLIENT_H
#define CENTRAL_UDP_CLIENT_H

#include <lwip/sockets.h>
#include <esp_event.h>

class UdpClient {
public:
    void init();
    void connect_to_wifi();
    static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,void *event_data);
    static void udp_timer_callback(TimerHandle_t xTimer);
    static void udp_send_task(void *pvParameters);

    int udp_socket;
    struct sockaddr_in udp_server_addr;
    TimerHandle_t udp_timer_handle;
};

inline UdpClient udp_client;

#endif //CENTRAL_UDP_CLIENT_H

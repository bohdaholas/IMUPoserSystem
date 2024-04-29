#ifndef CENTRAL_WIFI_H
#define CENTRAL_WIFI_H

#include <esp_event.h>

#ifdef __cplusplus
extern "C" {
#endif

class Wifi {
    public:
    void init();
    void wait_till_connected();

    SemaphoreHandle_t is_connected_semaphore;
private:
    void connect_to_wifi();
    static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id,
                                   void *event_data);
};

inline Wifi wifi;

#ifdef __cplusplus
}
#endif

#endif //CENTRAL_WIFI_H

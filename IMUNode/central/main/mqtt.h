#ifndef CENTRAL_MQTT_H
#define CENTRAL_MQTT_H

#include <mqtt_client.h>

#ifdef __cplusplus
extern "C" {
#endif

class MQTT {
public:
    void init();
    static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
    static void pub_msg_on_btn_press(void *pvParameters);

    static void send_tpose_req_cb();
private:
    esp_mqtt_client_handle_t mqtt_client = nullptr;
};

inline MQTT mqtt_manager;

#ifdef __cplusplus
}
#endif

#endif //CENTRAL_MQTT_H

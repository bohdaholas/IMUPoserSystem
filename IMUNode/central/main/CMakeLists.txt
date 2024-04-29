idf_component_register(
    SRCS "main.cpp" "gap.cpp" "gatt.cpp" "misc.c" "peer.c" "wifi.cpp" "udp_client.cpp" "mqtt.cpp"
    INCLUDE_DIRS "."
    REQUIRES nvs_utils
    REQUIRES esp_wifi
    REQUIRES bt
    REQUIRES imu
    REQUIRES mqtt
)

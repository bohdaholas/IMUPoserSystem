idf_component_register(
    SRCS "main.cpp" "gap.cpp" "gatt.cpp" "misc.c" "peer.c" "wifi.cpp" "udp_client.cpp" "mqtt.cpp"
    INCLUDE_DIRS "."

    # EspIDF components
    REQUIRES esp_wifi
    REQUIRES bt
    REQUIRES mqtt

    # Custom components in common directory
    REQUIRES nvs_utils
    REQUIRES button
    REQUIRES led
    REQUIRES imu
)

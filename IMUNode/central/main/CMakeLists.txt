idf_component_register(
    SRCS "main.cpp" "gap.cpp" "gatt.cpp" "misc.c" "peer.c" "udp_client.cpp"
    INCLUDE_DIRS "."
    REQUIRES nvs_utils
    REQUIRES esp_wifi
    REQUIRES bt
    REQUIRES imu
)

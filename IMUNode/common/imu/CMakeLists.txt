idf_component_register(
    SRCS "imu.cpp"
    INCLUDE_DIRS "."
    REQUIRES button
    REQUIRES nvs_utils
    REQUIRES eigen
    REQUIRES BNO055ESP32
)

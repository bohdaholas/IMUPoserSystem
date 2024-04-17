idf_component_register(
    SRCS "imu.cpp"
    INCLUDE_DIRS "."
    REQUIRES eigen
    REQUIRES BNO055ESP32 
)

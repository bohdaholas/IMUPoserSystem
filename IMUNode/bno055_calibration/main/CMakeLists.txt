idf_component_register(
    SRCS "bno055_calibration.cpp"
    INCLUDE_DIRS "."
    REQUIRES nvs_utils
    REQUIRES imu
)

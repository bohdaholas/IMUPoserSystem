idf_component_register(
        SRCS "raw_measurements_stream.cpp"
        INCLUDE_DIRS "."
        REQUIRES imu
        REQUIRES nvs_utils
)

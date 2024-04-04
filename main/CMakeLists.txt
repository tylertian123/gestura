idf_component_register(SRCS "flex_sensor.cpp" "gpio.cpp" "main.cpp"
                    INCLUDE_DIRS ".")

add_compile_definitions("ESP32C3_IMU_CONFIG")

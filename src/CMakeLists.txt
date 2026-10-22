idf_component_register(SRCS "main.cpp" "can_bus.cpp" "debug_led.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_twai)

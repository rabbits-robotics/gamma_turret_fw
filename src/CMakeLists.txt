idf_component_register(SRCS "main.cpp" "comm.cpp" "control.cpp" "can_sched.cpp" "imu_task.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_uart esp_timer rabcl rabcl_esp32)

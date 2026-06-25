idf_component_register(SRCS "uart.c" "web_control.c"
                       INCLUDE_DIRS "."
                       REQUIRES wifi_module esp_http_server uart_module motor_module camera_module imu_usb nvs_flash esp_timer)

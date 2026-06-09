idf_component_register(
    SRCS "camera_module.c"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_video
        esp_driver_jpeg
        driver
)

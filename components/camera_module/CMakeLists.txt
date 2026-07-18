idf_component_register(
    SRCS "camera_module.c"
    INCLUDE_DIRS "include" "."
    REQUIRES
        esp_video
        esp_driver_jpeg
        driver
        esp_timer
        esp_mm
)
target_compile_options(__idf_camera_module PRIVATE -Wno-error=misleading-indentation)

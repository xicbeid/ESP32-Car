idf_component_register(
    SRCS "face_recog.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp-dl spi_flash spiffs human_detect
)
# Compact code format needs this warning relaxed
target_compile_options(__idf_face_recog PRIVATE -Wno-error=misleading-indentation)

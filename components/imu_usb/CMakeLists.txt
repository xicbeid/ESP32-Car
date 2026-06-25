idf_component_register(SRCS "imu_usb.c" "imu_parser.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES usb)

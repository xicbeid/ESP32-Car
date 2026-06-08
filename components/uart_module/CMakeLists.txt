idf_component_register(
    SRCS "uart_module.c"
    INCLUDE_DIRS "."
    REQUIRES driver
    PRIV_REQUIRES motor_module
)
idf_component_register(SRCS "modbus_params.c" "tcp_slave.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp-modbus)
                    
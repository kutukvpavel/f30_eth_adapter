idf_component_register(SRCS "f30.cpp" "main.cpp"
                            "dbg_console.cpp"
                            "params.cpp"
                            "my_hal.cpp"
                            "modbus.cpp"
                            "f30.cpp"
                            "esp_linenoise_shim.c"
                        PRIV_REQUIRES esp_netif 
                            esp_eth 
                            nvs_flash 
                            esp_driver_gpio 
                            esp_driver_uart 
                            esp-modbus
                            console
                            spiffs
                            spi_flash
                        REQUIRES my_modbus ethernet_init macros periodic_task esp_eth_console meter_web_server
                       INCLUDE_DIRS ".")

idf_component_register(SRCS "ethernet_init.c" "eth_mdns_init.c"
                       PRIV_REQUIRES esp_driver_gpio esp_eth mdns macros
                       INCLUDE_DIRS ".")

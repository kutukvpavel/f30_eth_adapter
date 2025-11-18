idf_component_register(SRCS "eth_console.c" "eth_console_vfs.cpp"
                       PRIV_REQUIRES vfs lwip esp_ringbuf
                       INCLUDE_DIRS ".")

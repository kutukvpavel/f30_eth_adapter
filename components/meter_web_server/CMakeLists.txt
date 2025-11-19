idf_component_register(
    SRCS "meter_web_server.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_http_server
    EMBED_FILES "root.html"
    )

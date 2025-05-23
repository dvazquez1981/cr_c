idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES  esp_system esp_event esp-mqtt spi_flash driver esp_modem esp_netif esp_http_client esp_modem 
)


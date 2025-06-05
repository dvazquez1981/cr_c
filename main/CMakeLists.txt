idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES  esp_system esp_event  spi_flash driver  esp_netif esp_http_client 
)


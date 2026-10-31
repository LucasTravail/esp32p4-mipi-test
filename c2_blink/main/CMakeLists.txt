idf_component_register(SRCS "blink.c" REQUIRES esp_driver_gpio esp_driver_uart esp_http_server esp_wifi esp_netif esp_event nvs_flash bt)

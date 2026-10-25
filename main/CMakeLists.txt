idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_lcd esp_mm esp_driver_gpio esp_driver_i2c soc esp_system lvgl__lvgl)

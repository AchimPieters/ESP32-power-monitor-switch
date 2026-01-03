idf_component_register(
    SRCS "main.c" "esp32-lcm.c"
    INCLUDE_DIRS "."
    REQUIRES freertos esp_wifi esp_event esp_netif nvs_flash driver esp_timer esp_system homekit button bl0942 custom_charcteristcs
)

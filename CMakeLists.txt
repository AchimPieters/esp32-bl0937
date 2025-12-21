cmake_minimum_required(VERSION 3.16)

idf_component_register(
    SRCS "src/bl0973.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver esp_timer nvs_flash
)

set(srcs "main.c" "nimble_spp_server.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash
                       PRIV_REQUIRES bt esp_driver_uart)

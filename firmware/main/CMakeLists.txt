idf_component_register(
    SRCS "main.c" "mishka_wake.cpp" "mishka_assist.c"
    EMBED_FILES "mishka.tflite"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_i2c esp_driver_i2s esp_driver_rmt esp_codec_dev
             esp_wifi esp_event esp_netif app_update esp_partition nvs_flash esp_http_client esp_http_server esp_websocket_client mbedtls mqtt esp_audio_codec json espressif__esp-tflite-micro mishka_frontend
)

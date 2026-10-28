set(MAIN_SRCS "main.c" "config_manager.c" "ble_config_server.c" "whip_client.c"
              "media_codec.c" "webrtc_session.c" "audio_codec.c" "sco_bridge.c")
if(CONFIG_GW_NET_ETHERNET)
    list(APPEND MAIN_SRCS "ethernet_manager.c")
endif()
idf_component_register(SRCS "${MAIN_SRCS}"
                       INCLUDE_DIRS "."
                       REQUIRES esp_http_client esp-tls nvs_flash esp_wifi bt
                                esp_eth esp_netif esp_app_format
                                esp_driver_gpio esp_coex)

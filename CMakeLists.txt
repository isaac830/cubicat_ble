idf_component_register(SRCS "ble_server.cpp" "ble_client.cpp"
                        REQUIRES bt nvs_flash
                       INCLUDE_DIRS ".")
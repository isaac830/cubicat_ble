file(GLOB_RECURSE SRCS "ble_server.cpp" "ble_client.cpp" "./*.cpp")

idf_component_register(SRCS ${SRCS}
                        REQUIRES bt nvs_flash
                       INCLUDE_DIRS ".")
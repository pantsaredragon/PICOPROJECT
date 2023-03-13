add_executable(kemetVibrationSensor
kemetVibrationSensor.c
)
target_include_directories(kemetVibrationSensor PRIVATE  ../../AppData/Local/VisualGDB/PicoSDK/1.4.0-Package/examples/pico_w ../../AppData/Local/VisualGDB/PicoSDK/1.4.0-Package/examples/pico_w/access_point ../../AppData/Local/VisualGDB/PicoSDK/1.4.0-Package/lib/cyw43-driver/src ../../AppData/Local/VisualGDB/PicoSDK/1.4.0-Package/lib/lwip/contrib/ports/unix/port/include ../../AppData/Local/VisualGDB/PicoSDK/1.4.0-Package/lib/lwip/src/include ../../AppData/Local/VisualGDB/PicoSDK/1.4.0-Package/src/rp2_common/pico_cyw43_arch/include )

# pull in common dependencies and additional hardware support
target_link_libraries(kemetVibrationSensor
                pico_cyw43_arch_lwip_poll
                pico_stdlib
                pico_lwip_mbedtls
                pico_mbedtls
                pico_lwip_mqtt
                pico_multicore
                hardware_adc
            )

target_compile_definitions(kemetVibrationSensor PRIVATE
            WIFI_SSID=\"${WIFI_SSID}\"
            WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
            TEST_TCP_SERVER_IP=\"${TEST_TCP_SERVER_IP}\"
            NO_SYS=1
            )

 target_include_directories(kemetVibrationSensor PRIVATE
                ${CMAKE_CURRENT_LIST_DIR}
                ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
            )
# enable usb output, disable uart output
pico_enable_stdio_usb(kemetVibrationSensor 1)
pico_enable_stdio_uart(kemetVibrationSensor 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(kemetVibrationSensor)


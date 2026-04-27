file(GLOB_RECURSE SOURCES "src/*.c" "src/*.cpp")

# Conditionally exclude audio source files if audio is disabled
if(NOT CONFIG_CUBE32_AUDIO_ENABLED)
    list(FILTER SOURCES EXCLUDE REGEX "src/drivers/audio/.*")
endif()

# Conditionally exclude BLE OTA source files if BLE OTA is disabled
if(NOT CONFIG_CUBE32_BLE_OTA_ENABLED)
    list(FILTER SOURCES EXCLUDE REGEX "src/drivers/ble/.*")
endif()

# Conditionally exclude IMU source files if IMU is disabled
if(NOT CONFIG_CUBE32_IMU_ENABLED)
    list(FILTER SOURCES EXCLUDE REGEX "src/drivers/imu/.*")
endif()

# Conditionally exclude Magnetometer source files if MAG is disabled
if(NOT CONFIG_CUBE32_MAG_ENABLED)
    list(FILTER SOURCES EXCLUDE REGEX "src/drivers/mag/.*")
endif()

# Build list of required components
set(PRIV_REQUIRED_COMPONENTS
    esp_lcd 
    esp_lcd_touch
    esp_lcd_touch_cst816s
    esp_lcd_touch_ft5x06
    esp_driver_gpio 
    esp_driver_i2c 
    esp_driver_spi
    esp_driver_sdmmc
    esp_driver_i2s
    esp_driver_ledc
    esp_timer
    nvs_flash
    esp_event
    esp_netif
    spi_flash
    esp_io_expander
    esp_io_expander_tca9554
    esp_modem
    esp_modem_usb_dte
    usb_host_cdc_acm
    app_update
)

# Always add bt component for NimBLE BLE OTA support
# The bt component is required when BLE sources are compiled
list(APPEND PRIV_REQUIRED_COMPONENTS bt)

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "include" "include/utils" "include/drivers"
    PRIV_REQUIRES ${PRIV_REQUIRED_COMPONENTS}
    REQUIRES
        lvgl
        esp_lvgl_port
        cube32esp__xpowerslib
        esp32-camera
        fatfs
        sdmmc
        esp_codec_dev
        chmorgan__esp-libhelix-mp3
        esp_adc
        button
        usb
)

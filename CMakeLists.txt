file(GLOB_RECURSE SOURCES "src/*.c" "src/*.cpp")

# Conditionally exclude audio source files if audio is disabled
if(NOT CONFIG_CUBE32_AUDIO_ENABLED)
    list(FILTER SOURCES EXCLUDE REGEX "src/drivers/audio/.*")
endif()

# Conditionally exclude BLE OTA source files if BLE OTA is disabled OR if
# NimBLE is not enabled. BT_NIMBLE_ENABLED is a Kconfig choice symbol that
# cannot be set via `select` — it must be set in sdkconfig before building.
if(NOT CONFIG_CUBE32_BLE_OTA_ENABLED OR NOT CONFIG_BT_NIMBLE_ENABLED)
    list(FILTER SOURCES EXCLUDE REGEX "src/drivers/ble/.*")
    if(CONFIG_CUBE32_BLE_OTA_ENABLED AND NOT CONFIG_BT_NIMBLE_ENABLED)
        message(WARNING "CUBE32: BLE OTA enabled but BT_NIMBLE_ENABLED is not set -- "
                        "BLE OTA sources will not be compiled. "
                        "Enable NimBLE in menuconfig: "
                        "Component config -> Bluetooth -> Bluetooth -> NimBLE - BLE only.")
    endif()
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

# Always add bt when BLE OTA is enabled. In MINIMAL_BUILD mode, conditional
# PRIV_REQUIRES are resolved during the component scan before sdkconfig is
# fully loaded, so an unconditional dep is safer. Doing it inside the BLE OTA
# guard is sufficient since bt is only needed when BLE OTA compiles.
if(CONFIG_CUBE32_BLE_OTA_ENABLED AND CONFIG_BT_NIMBLE_ENABLED)
    list(APPEND PRIV_REQUIRED_COMPONENTS bt)
endif()

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

# idf_component_register's PRIV_REQUIRES does not reliably propagate include
# directories from bt in MINIMAL_BUILD mode (ESP-IDF resolves component deps
# before sdkconfig is fully loaded on the first cmake pass, so bt may not yet
# be known as a dep of cube32_bsp when include paths are assembled).
# We therefore pull bt's public include dirs explicitly after registration,
# resolving relative entries against bt's own COMPONENT_DIR.
if(CONFIG_CUBE32_BLE_OTA_ENABLED AND CONFIG_BT_NIMBLE_ENABLED)
    idf_component_get_property(bt_component_dir bt COMPONENT_DIR)
    idf_component_get_property(bt_include_dirs  bt INCLUDE_DIRS)
    set(_bt_resolved_includes "")
    foreach(_inc ${bt_include_dirs})
        if(IS_ABSOLUTE "${_inc}")
            list(APPEND _bt_resolved_includes "${_inc}")
        else()
            list(APPEND _bt_resolved_includes "${bt_component_dir}/${_inc}")
        endif()
    endforeach()
    if(_bt_resolved_includes)
        target_include_directories(__idf_cube32_bsp PRIVATE ${_bt_resolved_includes})
    else()
        # Fallback if bt properties are not yet set: add the well-known
        # NimBLE porting paths directly from IDF_PATH.
        target_include_directories(__idf_cube32_bsp PRIVATE
            "$ENV{IDF_PATH}/components/bt/host/nimble/nimble/porting/nimble/include"
            "$ENV{IDF_PATH}/components/bt/host/nimble/nimble/nimble/include"
            "$ENV{IDF_PATH}/components/bt/host/nimble/port/include"
            "$ENV{IDF_PATH}/components/bt/host/nimble/nimble/porting/npl/freertos/include"
        )
    endif()
    # Also link against the bt library so NimBLE symbols resolve at link time.
    # PRIV_REQUIRES propagation is unreliable in MINIMAL_BUILD for the same
    # reason as the include dirs above.
    target_link_libraries(__idf_cube32_bsp PRIVATE idf::bt)
endif()


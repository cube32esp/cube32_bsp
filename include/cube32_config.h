/**
 * @file cube32_config.h
 * @brief CUBE32 Board Configuration - Hardware Pin Definitions
 * 
 * This file contains all hardware pin definitions and configuration
 * constants for the CUBE32 board variants.
 */

#ifndef CUBE32_CONFIG_H
#define CUBE32_CONFIG_H

#include <driver/gpio.h>
#include <driver/i2c_types.h>
#include <driver/spi_common.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Board Variant Selection
 * ============================================================================ */

// Define board variants
#define CUBE32_VARIANT_BASIC      0
#define CUBE32_VARIANT_PRO        1
#define CUBE32_VARIANT_AI         2

// Default to BASIC variant if not specified
#ifndef CUBE32_VARIANT
#define CUBE32_VARIANT CUBE32_VARIANT_BASIC
#endif

/* ============================================================================
 * I2C Configuration
 * ============================================================================ */

#define CUBE32_I2C_NUM            I2C_NUM_0
#define CUBE32_I2C_SDA_PIN        GPIO_NUM_17
#define CUBE32_I2C_SCL_PIN        GPIO_NUM_18
#define CUBE32_I2C_FREQ_HZ        400000

/* ============================================================================
 * SPI Configuration
 * ============================================================================ */

#define CUBE32_SPI_HOST           SPI2_HOST
#define CUBE32_SPI_MOSI_PIN       GPIO_NUM_0
#define CUBE32_SPI_MISO_PIN       GPIO_NUM_4
#define CUBE32_SPI_SCLK_PIN       GPIO_NUM_1
#define CUBE32_SPI_FREQ_HZ        40000000

/* ============================================================================
 * Display Configuration
 * ============================================================================ */

#define CUBE32_LCD_CS_PIN         GPIO_NUM_46
#define CUBE32_LCD_DC_PIN         GPIO_NUM_2
#define CUBE32_LCD_RST_PIN        GPIO_NUM_NC
#define CUBE32_LCD_BL_PIN         GPIO_NUM_NC

/* LCD Resolution based on display board model from Kconfig */
#if defined(CONFIG_CUBE32_DISPLAY_CUBE_TFT_TOUCH_154)
#define CUBE32_LCD_H_RES          240
#define CUBE32_LCD_V_RES          240
#define CUBE32_LCD_DEFAULT_ROTATION  270     /* 1.54" default rotation */
#elif defined(CONFIG_CUBE32_DISPLAY_CUBE_TFT_TOUCH_200)
#define CUBE32_LCD_H_RES          240
#define CUBE32_LCD_V_RES          320
#define CUBE32_LCD_DEFAULT_ROTATION  0     /* 2.0" default rotation */
#else
/* Default resolution if no board model selected */
#define CUBE32_LCD_H_RES          240
#define CUBE32_LCD_V_RES          240
#define CUBE32_LCD_DEFAULT_ROTATION  0
#endif

#define CUBE32_LCD_PIXEL_CLK_HZ   (40 * 1000 * 1000)

/* ============================================================================
 * Touch Configuration
 * ============================================================================ */

#define CUBE32_TOUCH_I2C_NUM      CUBE32_I2C_NUM
#define CUBE32_TOUCH_RST_PIN      GPIO_NUM_NC
#define CUBE32_TOUCH_INT_PIN      GPIO_NUM_NC

/* ============================================================================
 * Audio Configuration
 * ============================================================================ */

#define CUBE32_AUDIO_I2S_NUM       CUBE32_I2C_NUM
#define CUBE32_AUDIO_I2S_MCLK_PIN  GPIO_NUM_16
#define CUBE32_AUDIO_I2S_LRCK_PIN  GPIO_NUM_45
#define CUBE32_AUDIO_I2S_BCLK_PIN  GPIO_NUM_9
#define CUBE32_AUDIO_I2S_DO_PIN    GPIO_NUM_8
#define CUBE32_AUDIO_I2S_DI_PIN    GPIO_NUM_10
#define CUBE32_AUDIO_PA_PIN        GPIO_NUM_NC

// AEC mode is configured via Kconfig (CUBE32_AUDIO_AEC_MODE choice)

#define CUBE32_AUDIO_CODEC_ES8311_ADDR  ES8311_CODEC_DEFAULT_ADDR
#define CUBE32_AUDIO_CODEC_ES7210_ADDR  ES7210_CODEC_DEFAULT_ADDR

#define CUBE32_AUDIO_IOX_ADDR       0x20

/* Audio IO Expander (TCA9554 @ CUBE32_AUDIO_IOX_ADDR) Port Assignment */
#define CUBE32_AUDIO_IOX_PA_CTRL_PIN    0       ///< Port 0: PA amplifier control (Output)
#define CUBE32_AUDIO_IOX_PJ_DET_PIN     1       ///< Port 1: Phone jack detect (Input, LOW=plugged)

#define CUBE32_ADC_BUTTON_PIN       GPIO_NUM_5

/* ----------------------------------------------------------------------------
 * ADC Button Voltage Ranges (mV)
 * 
 * Resistor divider from 3.3V with 10K pull-up:
 * - Button 0: 10K + 1.3K  -> ~0.38V (380mV)
 * - Button 1: 10K + 3.3K  -> ~0.82V (820mV)
 * - Button 2: 10K + 5.1K  -> ~1.11V (1110mV)
 * - Button 3: 10K + 10K   -> ~1.65V (1650mV)
 * - Button 4: 10K + 15K   -> ~1.98V (1980mV)
 * - Button 5: 10K + 27K   -> ~2.41V (2410mV)
 * 
 * Voltage ranges adjusted for reliable detection with proper gaps.
 * Ranges should not overlap to avoid false triggers.
 * ---------------------------------------------------------------------------- */

/* Button 0: 1.3K resistor, ~0.38V */
#define CUBE32_ADC_BUTTON_0_MIN_MV   180   //100
#define CUBE32_ADC_BUTTON_0_MAX_MV   580    //600

/* Button 1: 3.3K resistor, ~0.82V */
#define CUBE32_ADC_BUTTON_1_MIN_MV   600    //650
#define CUBE32_ADC_BUTTON_1_MAX_MV   960    //1000

/* Button 2: 5.1K resistor, ~1.11V */
#define CUBE32_ADC_BUTTON_2_MIN_MV   965    //1050
#define CUBE32_ADC_BUTTON_2_MAX_MV   1255   //1350

/* Button 3: 10K resistor, ~1.65V */
#define CUBE32_ADC_BUTTON_3_MIN_MV   1490   //1450
#define CUBE32_ADC_BUTTON_3_MAX_MV   1810   //1850

/* Button 4: 15K resistor, ~1.98V */
#define CUBE32_ADC_BUTTON_4_MIN_MV   1815   //1900
#define CUBE32_ADC_BUTTON_4_MAX_MV   2145   //2200

/* Button 5: 27K resistor, ~2.41V */
#define CUBE32_ADC_BUTTON_5_MIN_MV   2200   //2250
#define CUBE32_ADC_BUTTON_5_MAX_MV   2620   //2650

/* ============================================================================
 * Camera Configuration
 * ============================================================================ */

#define CUBE32_CAM_PWDN_PIN       GPIO_NUM_NC
#define CUBE32_CAM_RESET_PIN      GPIO_NUM_NC
#define CUBE32_CAM_XCLK_PIN       GPIO_NUM_6
#define CUBE32_CAM_SIOD_PIN       CUBE32_I2C_SDA_PIN
#define CUBE32_CAM_SIOC_PIN       CUBE32_I2C_SCL_PIN
#define CUBE32_CAM_D7_PIN         GPIO_NUM_39
#define CUBE32_CAM_D6_PIN         GPIO_NUM_41
#define CUBE32_CAM_D5_PIN         GPIO_NUM_42
#define CUBE32_CAM_D4_PIN         GPIO_NUM_12
#define CUBE32_CAM_D3_PIN         GPIO_NUM_3
#define CUBE32_CAM_D2_PIN         GPIO_NUM_14
#define CUBE32_CAM_D1_PIN         GPIO_NUM_40
#define CUBE32_CAM_D0_PIN         GPIO_NUM_13
#define CUBE32_CAM_VSYNC_PIN      GPIO_NUM_21
#define CUBE32_CAM_HREF_PIN       GPIO_NUM_38
#define CUBE32_CAM_PCLK_PIN       GPIO_NUM_11

#define CUBE32_CAM_XCLK_FREQ_HZ   20000000

/* ============================================================================
 * SD Card Configuration
 * ============================================================================ */

#define CUBE32_SD_CMD_PIN         GPIO_NUM_7
#define CUBE32_SD_CLK_PIN         GPIO_NUM_15
#define CUBE32_SD_D0_PIN          GPIO_NUM_4

/* ============================================================================
 * PMU Configuration (if applicable)
 * ============================================================================ */

#define CUBE32_PMU_I2C_ADDR       0x34

/* ============================================================================
 * RTC Configuration (BM8563)
 * ============================================================================ */

#define CUBE32_RTC_I2C_ADDR       0x51

/* ============================================================================
 * Sensor Axis Mapping Configuration
 *
 * Remap chip-level XYZ to board-level XYZ for different sensor placement.
 * Each axis has a sign multiplier (+1 or -1).  Verified empirically for
 * the default CUBE32 board layout via multi-orientation rotation tests.
 *
 * To adapt for a new board variant, change only the sign values below.
 * ============================================================================ */

/* IMU (LSM6DSO) — applies to both accelerometer and gyroscope */
#define CUBE32_IMU_AXIS_X_SIGN   (-1)   /* board_x = -chip_x */
#define CUBE32_IMU_AXIS_Y_SIGN   (-1)   /* board_y = -chip_y */
#define CUBE32_IMU_AXIS_Z_SIGN   (+1)   /* board_z = +chip_z */

/* Magnetometer (LIS3MDL) */
#define CUBE32_MAG_AXIS_X_SIGN   (+1)   /* board_x = +chip_x */
#define CUBE32_MAG_AXIS_Y_SIGN   (-1)   /* board_y = -chip_y */
#define CUBE32_MAG_AXIS_Z_SIGN   (+1)   /* board_z = +chip_z */

/* Default magnetometer hard iron calibration offsets (gauss).
 * Used when no calibration data is stored in NVS. */
#define CUBE32_MAG_CAL_OFFSET_X  (-0.728f)
#define CUBE32_MAG_CAL_OFFSET_Y  ( 8.499f)
#define CUBE32_MAG_CAL_OFFSET_Z  ( 3.716f)

/* ============================================================================
 * Modem Configuration (A7670)
 * ============================================================================ */

/* A7670 IO Expander (TCA9554) I2C Address */
#define CUBE32_MODEM_IOX_ADDR     0x22

/* A7670 IO Expander Pin Assignments */
#define CUBE32_MODEM_IOX_PWRKEY_PIN      0   // A7670 PWRKEY PIN
#define CUBE32_MODEM_IOX_DTR_PIN         1   // A7670 DTR PIN: idle high
#define CUBE32_MODEM_IOX_PWR_RAIL_PIN    2   // A7670 Power Rail: L-Off, H-On
#define CUBE32_MODEM_IOX_COMM_SEL_PIN    3   // MCU-A7670 Comm Port: L-USB, H-UART

/* A7670 USB Descriptor (fixed by SimCom firmware) */
#define CUBE32_MODEM_USB_VID             0x1E0E  // SimCom USB Vendor ID
#define CUBE32_MODEM_USB_PID             0x9011  // A7670 USB Product ID
#define CUBE32_MODEM_USB_AT_INTERFACE    4       // USB interface index for AT commands
#define CUBE32_MODEM_USB_DATA_INTERFACE  5       // USB interface index for PPP data

/* A7670 UART Configuration
 * GPIO 19/20 are shared with ESP32-S3 USB D-/D+ pins.
 * When UART mode is selected, these pins are reconfigured as GPIO via gpio_reset_pin().
 * UART0 is reserved for console, UART2 is left available for extensions. */
#define CUBE32_MODEM_UART_TX_PIN         GPIO_NUM_20  // ESP TX → A7670 RXD
#define CUBE32_MODEM_UART_RX_PIN         GPIO_NUM_19  // ESP RX ← A7670 TXD
#define CUBE32_MODEM_UART_PORT_NUM       1            // UART1 (UART0=console, UART2=available)
#define CUBE32_MODEM_UART_BAUD_RATE_DEF  115200

/* Modem timing constants (ms) */
#define CUBE32_MODEM_PWRKEY_ON_TIME_MS       500     // Pulse time to turn on (A7670 needs 500ms+ for cold boot)
#define CUBE32_MODEM_PWRKEY_OFF_TIME_MS      2600    // Pulse time to turn off (inverted: HIGH on IOX)
#define CUBE32_MODEM_PWRKEY_SETTLE_MS        100     // Settle time before/after pulse
#define CUBE32_MODEM_POWERON_WAIT_MS         10000   // Wait time after power on sequence
#define CUBE32_MODEM_PWR_RAIL_SETTLE_MS      500     // Power rail settle time (cold boot needs longer)
#define CUBE32_MODEM_COLD_BOOT_DELAY_MS      1000    // Delay before modem init for cold boot stabilization

#ifdef __cplusplus
}
#endif

#endif /* CUBE32_CONFIG_H */

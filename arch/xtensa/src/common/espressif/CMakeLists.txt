# ##############################################################################
# arch/xtensa/src/common/espressif/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS)

if(CONFIG_ESP_RMT)
  list(APPEND SRCS esp_rmt.c)
  if(CONFIG_DRIVERS_RC)
    list(APPEND SRCS esp_lirc.c)
  endif()
  if(CONFIG_WS2812_NON_SPI_DRIVER)
    list(APPEND SRCS esp_ws2812.c)
  endif()
endif()

if(CONFIG_ESP_MCPWM)
  list(APPEND SRCS esp_mcpwm.c)
endif()

if(CONFIG_ESPRESSIF_SIMPLE_BOOT
   OR CONFIG_ESP32_APP_FORMAT_MCUBOOT
   OR CONFIG_ESP32S2_APP_FORMAT_MCUBOOT
   OR CONFIG_ESP32S3_APP_FORMAT_MCUBOOT)
  list(APPEND SRCS esp_loader.c)
endif()

if(CONFIG_ESPRESSIF_TEMP)
  list(APPEND SRCS esp_temperature_sensor.c)
endif()

if(CONFIG_ESP_SDM)
  list(APPEND SRCS esp_sdm.c)
endif()

if(CONFIG_ESPRESSIF_I2S)
  list(APPEND SRCS esp_i2s.c)
endif()

if(CONFIG_ESPRESSIF_I2C_BITBANG)
  list(APPEND SRCS esp_i2c_bitbang.c)
endif()

if(CONFIG_ESPRESSIF_SPI_BITBANG)
  list(APPEND SRCS esp_spi_bitbang.c)
endif()

if(CONFIG_ESP_PCNT)
  list(APPEND SRCS esp_pcnt.c)
  if(CONFIG_ESP_PCNT_AS_QE)
    list(APPEND SRCS esp_qencoder.c)
  endif()
endif()

if(CONFIG_ESPRESSIF_LEDC)
  list(APPEND SRCS esp_ledc.c)
endif()

if(CONFIG_ESPRESSIF_SPIFLASH)
  list(APPEND SRCS esp_spiflash.c)
  if(CONFIG_ESPRESSIF_MTD)
    list(APPEND SRCS esp_spiflash_mtd.c)
  endif()
endif()

if(CONFIG_ESPRESSIF_I2C_PERIPH_SLAVE_MODE)
  list(APPEND SRCS esp_i2c_slave.c)
endif()

if(CONFIG_ESPRESSIF_DEDICATED_GPIO)
  list(APPEND SRCS esp_dedic_gpio.c)
endif()

if(CONFIG_ESPRESSIF_SHA_ACCELERATOR)
  list(APPEND SRCS esp_sha.c)
endif()

if(CONFIG_ESPRESSIF_AES_ACCELERATOR)
  list(APPEND SRCS esp_aes.c)
endif()

if(NOT CONFIG_ARCH_CHIP_ESP32 AND CONFIG_CRYPTO_CRYPTODEV_HARDWARE)
  list(APPEND SRCS esp_crypto.c)
endif()

if(CONFIG_ESPRESSIF_USE_ULP_RISCV_CORE)
  list(APPEND SRCS esp_ulp.c)
endif()

if(CONFIG_SYSTEM_NXDIAG_ESPRESSIF_CHIP_WO_TOOL)
  list(APPEND SRCS esp_nxdiag.c)
endif()

if(CONFIG_ESPRESSIF_WIRELESS)
  list(APPEND SRCS esp_wireless.c esp_wifi_utils.c)
  if(CONFIG_ESPRESSIF_WIFI)
    list(APPEND SRCS esp_wifi_event_handler.c esp_wlan_netdev.c esp_wifi_api.c)
  endif()
  if(CONFIG_ESPRESSIF_ESPNOW_PKTRADIO)
    list(APPEND SRCS esp_espnow_pktradio.c)
  endif()
endif()

if(CONFIG_ESPRESSIF_ADC)
  list(APPEND SRCS esp_adc.c)
endif()

if(CONFIG_PM)
  if(NOT CONFIG_ARCH_CUSTOM_PMINIT)
    list(APPEND SRCS esp_pm_initialize.c)
  endif()
  list(APPEND SRCS esp_pm.c)
endif()

list(APPEND SRCS esp_efuse.c esp_irq.c esp_xtensa_intr.c esp_gpio.c)

if(CONFIG_RTC_DRIVER)
  list(APPEND SRCS esp_rtc.c)
endif()

if(CONFIG_ESPRESSIF_HR_TIMER)
  list(APPEND SRCS esp_timer_adapter.c)
endif()

target_compile_options(arch PRIVATE -Wno-shadow -Wno-undef)
target_link_options(nuttx PRIVATE "SHELL:-u esp_system_include_startup_funcs"
                    "SHELL:-u esp_timer_init_include_func")

if(CONFIG_ESPRESSIF_EFUSE)
  target_link_options(nuttx PRIVATE "SHELL:-u esp_efuse_startup_include_func")
endif()

target_include_directories(arch
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/platform_include)
target_sources(arch PRIVATE ${SRCS})

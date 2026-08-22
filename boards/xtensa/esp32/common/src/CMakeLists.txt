# ##############################################################################
# boards/xtensa/esp32/common/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS)

if(CONFIG_ARCH_BOARD_COMMON)
  if(CONFIG_WATCHDOG)
    list(APPEND SRCS esp32_board_wdt.c)
  endif()
  if(CONFIG_ESPRESSIF_ADC)
    list(APPEND SRCS esp32_board_adc.c)
  endif()
  if(CONFIG_ESP32_I2C)
    list(APPEND SRCS esp32_board_i2c.c)
  endif()
  if(CONFIG_ESP32_SPI)
    list(APPEND SRCS esp32_board_spi.c)
  endif()
  if(CONFIG_SPI_DRIVER)
    list(APPEND SRCS esp32_board_spidev.c)
  endif()
  if(CONFIG_SPI_SLAVE_DRIVER)
    list(APPEND SRCS esp32_board_spislavedev.c)
  endif()
  if(CONFIG_ESP32_SPIFLASH)
    list(APPEND SRCS esp32_board_spiflash.c)
  endif()
  if(CONFIG_ESPRESSIF_WIFI)
    list(APPEND SRCS esp32_board_wlan.c)
  endif()
endif()

target_sources(board PRIVATE ${SRCS})

# ##############################################################################
# boards/xtensa/esp32s2/common/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS)

if(CONFIG_ARCH_BOARD_COMMON)
  if(CONFIG_ESPRESSIF_ADC)
    list(APPEND SRCS esp32s2_board_adc.c)
  endif()
  if(CONFIG_WATCHDOG)
    list(APPEND SRCS esp32s2_board_wdt.c)
  endif()
  if(CONFIG_ESP32S2_SPI)
    list(APPEND SRCS esp32s2_board_spi.c)
  endif()
  if(CONFIG_SPI_DRIVER)
    list(APPEND SRCS esp32s2_board_spidev.c)
  endif()
  if(CONFIG_SPI_SLAVE_DRIVER)
    list(APPEND SRCS esp32s2_board_spislavedev.c)
  endif()
endif()

target_sources(board PRIVATE ${SRCS})

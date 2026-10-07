# ##############################################################################
# boards/risc-v/esp32c3-legacy/common/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS)

if(CONFIG_ARCH_BOARD_COMMON)
  if(CONFIG_ESP32C3_SPI)
    list(APPEND SRCS esp32c3_board_spi.c)
  endif()
  if(CONFIG_SPI_DRIVER)
    list(APPEND SRCS esp32c3_board_spidev.c)
  endif()
  if(CONFIG_SPI_SLAVE_DRIVER)
    list(APPEND SRCS esp32c3_board_spislavedev.c)
  endif()
  if(CONFIG_WATCHDOG)
    list(APPEND SRCS esp32c3_board_wdt.c)
  endif()
endif()

target_sources(board PRIVATE ${SRCS})

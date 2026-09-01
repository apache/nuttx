# ##############################################################################
# boards/xtensa/esp32s3/common/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS)

if(CONFIG_ARCH_BOARD_COMMON)
  if(CONFIG_ESP32S3_TIMER)
    list(APPEND SRCS esp32s3_board_tim.c)
  endif()
  if(CONFIG_WATCHDOG)
    list(APPEND SRCS esp32s3_board_wdt.c)
  endif()
  if(CONFIG_ESP32S3_SPIFLASH)
    list(APPEND SRCS esp32s3_board_spiflash.c)
  endif()
  if(CONFIG_SPI_DRIVER)
    list(APPEND SRCS esp32s3_board_spidev.c)
  endif()
endif()

target_sources(board PRIVATE ${SRCS})

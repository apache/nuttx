# ##############################################################################
# boards/xtensa/esp32s3/esp32s3-touch-amoled-216/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS esp32s3_boot.c esp32s3_bringup.c)

if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS esp32s3_reset.c)
endif()

if(CONFIG_ESP32S3_SPI)
  list(APPEND SRCS esp32s3_board_spi.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS esp32s3_buttons.c)
endif()

target_sources(board PRIVATE ${SRCS})

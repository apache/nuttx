# ##############################################################################
# boards/xtensa/esp32s3/esp32s3-devkit/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS esp32s3_boot.c esp32s3_bringup.c)

if(CONFIG_BUILD_KERNEL)
  list(APPEND SRCS romfs_stub.c)
endif()

if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS esp32s3_reset.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS esp32s3_buttons.c)
endif()

if(CONFIG_INPUT_DJOYSTICK)
  list(APPEND SRCS esp32s3_djoystick.c)
endif()

if(CONFIG_ESP32S3_SPI)
  list(APPEND SRCS esp32s3_board_spi.c)
endif()

if(CONFIG_ESP32S3_TWAI)
  list(APPEND SRCS esp32s3_twai.c)
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS esp32s3_gpio.c)
endif()

if(CONFIG_LCD_ST7735)
  list(APPEND SRCS esp32s3_st7735.c)
endif()

target_sources(board PRIVATE ${SRCS})

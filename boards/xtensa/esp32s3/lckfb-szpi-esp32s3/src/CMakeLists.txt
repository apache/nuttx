# ##############################################################################
# boards/xtensa/esp32s3/lckfb-szpi-esp32s3/src/CMakeLists.txt
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

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS esp32s3_gpio.c)
endif()

if(CONFIG_IOEXPANDER_PCA9557)
  list(APPEND SRCS esp32s3_pca9557.c)
endif()

if(CONFIG_LCD)
  list(APPEND SRCS esp32s3_board_lcd.c)
endif()

if(CONFIG_INPUT_FT5X06)
  list(APPEND SRCS esp32s3_ft5x06.c)
endif()

if(CONFIG_ESP32S3_CAM)
  list(APPEND SRCS esp32s3_board_camera.c)
endif()

if(CONFIG_AUDIO_ES7210)
  list(APPEND SRCS esp32s3_board_es7210.c)
endif()

target_sources(board PRIVATE ${SRCS})

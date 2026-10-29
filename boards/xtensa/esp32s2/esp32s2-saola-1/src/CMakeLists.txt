# ##############################################################################
# boards/xtensa/esp32s2/esp32s2-saola-1/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS esp32s2_boot.c esp32s2_bringup.c)
if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS esp32s2_reset.c)
endif()
if(CONFIG_DEV_GPIO)
  list(APPEND SRCS esp32s2_gpio.c)
endif()
if(CONFIG_PWM)
  list(APPEND SRCS esp32s2_ledc.c)
endif()
if(CONFIG_ONESHOT)
  list(APPEND SRCS esp32s2_oneshot.c)
endif()
if(CONFIG_I2C_DRIVER)
  list(APPEND SRCS esp32s2_board_i2c.c)
endif()
if(CONFIG_SENSORS_BMP180)
  list(APPEND SRCS esp32s2_bmp180.c)
endif()
if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS esp32s2_buttons.c)
endif()

target_sources(board PRIVATE ${SRCS})

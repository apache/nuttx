# ##############################################################################
# boards/xtensa/esp32/esp32-m5-stickc-plus/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS esp32_boot.c esp32_bringup.c)

if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS esp32_reset.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS esp32_ledc.c)
endif()

if(CONFIG_USERLED)
  list(APPEND SRCS esp32_userleds.c)
endif()

if(CONFIG_ESP32_I2C0)
  list(APPEND SRCS esp32_axp192.c)
endif()

if(CONFIG_RTC_PCF8563)
  list(APPEND SRCS esp32_bm8563.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS esp32_buttons.c)
endif()

target_sources(board PRIVATE ${SRCS})

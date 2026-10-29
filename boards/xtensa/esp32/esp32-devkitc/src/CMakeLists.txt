# ##############################################################################
# boards/xtensa/esp32/esp32-devkitc/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS esp32_boot.c esp32_bringup.c)
if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS esp32_reset.c)
endif()
if(CONFIG_DEV_GPIO)
  list(APPEND SRCS esp32_gpio.c)
endif()
if(CONFIG_PWM)
  list(APPEND SRCS esp32_ledc.c)
endif()
if(CONFIG_USERLED)
  list(APPEND SRCS esp32_userleds.c)
endif()
if(CONFIG_WS2812)
  list(APPEND SRCS esp32_ws2812.c)
endif()
if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS esp32_buttons.c)
endif()
if(CONFIG_ESP32_TWAI)
  list(APPEND SRCS esp32_twai.c)
endif()
if(CONFIG_NET_W5500)
  list(APPEND SRCS esp32_w5500.c)
endif()

target_sources(board PRIVATE ${SRCS})

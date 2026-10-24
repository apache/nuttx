# ##############################################################################
# boards/xtensa/esp32/esp32-devkitc/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS esp32_boot.c esp32_bringup.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS esp32_appinit.c)
  if(CONFIG_BOARDCTL_RESET)
    list(APPEND SRCS esp32_reset.c)
  endif()
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS esp32_gpio.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS esp32_buttons.c)
endif()

target_sources(board PRIVATE ${SRCS})

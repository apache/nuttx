# ##############################################################################
# boards/xtensa/esp32s2/esp32s2-saola-1/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS esp32s2_boot.c esp32s2_bringup.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS esp32s2_appinit.c)
  if(CONFIG_BOARDCTL_RESET)
    list(APPEND SRCS esp32s2_reset.c)
  endif()
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS esp32s2_gpio.c)
endif()

target_sources(board PRIVATE ${SRCS})

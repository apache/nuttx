# ##############################################################################
# boards/risc-v/esp32c3-legacy/esp32c3-legacy-devkit/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS esp32c3_boot.c esp32c3_bringup.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS esp32c3_appinit.c)
  if(CONFIG_BOARDCTL_RESET)
    list(APPEND SRCS esp32c3_reset.c)
  endif()
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS esp32c3_gpio.c)
endif()

target_sources(board PRIVATE ${SRCS})

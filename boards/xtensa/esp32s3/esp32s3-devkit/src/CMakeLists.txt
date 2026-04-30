# ##############################################################################
# boards/xtensa/esp32s3/esp32s3-devkit/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

set(SRCS esp32s3_boot.c esp32s3_bringup.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS esp32s3_appinit.c)
  if(CONFIG_BOARDCTL_RESET)
    list(APPEND SRCS esp32s3_reset.c)
  endif()
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS esp32s3_gpio.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS esp32s3_buttons.c)
endif()

target_sources(board PRIVATE ${SRCS})

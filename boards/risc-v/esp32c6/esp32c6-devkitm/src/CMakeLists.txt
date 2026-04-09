# ##############################################################################
# boards/risc-v/esp32c6/esp32c6-devkitm/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

set(SRCS esp32c6_boot.c esp32c6_bringup.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS esp32c6_appinit.c)

  if(CONFIG_BOARDCTL_RESET)
    list(APPEND SRCS esp32c6_reset.c)
  endif()
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS esp32c6_gpio.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS esp32c6_buttons.c)
endif()

target_sources(board PRIVATE ${SRCS})

# ##############################################################################
# Linker scripts
# ##############################################################################

string(REPLACE "\"" "" CHIP_SERIES "${CONFIG_ESPRESSIF_CHIP_SERIES}")

set(BOARD_COMMON_DIR "${NUTTX_BOARD_DIR}/../common")

set(LDSCRIPTS "${BOARD_COMMON_DIR}/scripts/${CHIP_SERIES}_aliases.ld"
              "${BOARD_COMMON_DIR}/scripts/${CHIP_SERIES}_flat_memory.ld")

if(CONFIG_ESPRESSIF_BOOTLOADER_MCUBOOT OR CONFIG_ESPRESSIF_SIMPLE_BOOT)
  list(APPEND LDSCRIPTS
       "${BOARD_COMMON_DIR}/scripts/${CHIP_SERIES}_sections.ld")
else()
  list(APPEND LDSCRIPTS
       "${BOARD_COMMON_DIR}/scripts/${CHIP_SERIES}_legacy_sections.ld")
endif()

set_property(GLOBAL APPEND PROPERTY LD_SCRIPT ${LDSCRIPTS})

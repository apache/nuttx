# ##############################################################################
# boards/arm/rp23xx/pimoroni-pico-plus-2-w/src/CMakeLists.txt
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

set(SRCS rp23xx_boardinitialize.c rp23xx_bringup.c)

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS rp23xx_gpio.c)
endif()

# This board has no LED on an RP2350 pin -- its only LED hangs off the CYW43439,
# and reaching it means a gSPI bus transaction.  That rules out
# CONFIG_ARCH_LEDS, whose board_autoled_on() is called from interrupt handlers
# and from assertion handling, so there is no rp23xx_autoleds.c.  It also means
# there is no LED at all without the wireless chip.  See rp23xx_userleds.c.

if(CONFIG_ARCH_LEDS)
  message(
    FATAL_ERROR
      "CONFIG_ARCH_LEDS is not supported on this board: the LED hangs off the "
      "CYW43439 and cannot be driven from interrupt context.  "
      "Use CONFIG_USERLED instead.")
endif()

if(CONFIG_RP23XX_INFINEON_CYW43439)
  list(APPEND SRCS rp23xx_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS rp23xx_buttons.c)
endif()

# rp23xx_firmware.c, which carries the CYW43439 firmware blob, is built from
# boards/arm/rp23xx/common/src.

target_sources(board PRIVATE ${SRCS})

if(CONFIG_BOOT_RUNFROMFLASH)
  set_property(GLOBAL PROPERTY LD_SCRIPT
                               "${NUTTX_BOARD_DIR}/scripts/memmap_default.ld")
elseif(CONFIG_BOOT_COPYTORAM)
  set_property(
    GLOBAL PROPERTY LD_SCRIPT
                    "${NUTTX_BOARD_DIR}/scripts/memmap_copy_to_ram.ld")
else()
  set_property(GLOBAL PROPERTY LD_SCRIPT
                               "${NUTTX_BOARD_DIR}/scripts/memmap_no_flash.ld")
endif()

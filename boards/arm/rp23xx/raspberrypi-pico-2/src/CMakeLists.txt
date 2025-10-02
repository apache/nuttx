# ##############################################################################
# boards/arm/rp23xx/raspberrypi-pico-2/src/CMakeLists.txt
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

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS rp23xx_autoleds.c)
else()
  list(APPEND SRCS rp23xx_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS rp23xx_buttons.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS rp23xx_appinit.c)
endif()

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

if(CONFIG_ETC_ROMFS)
  # TODO: etc/init.d/rc.sysinit etc/init.d/rcS
endif()

# ##############################################################################
# boards/arm/stm32/photon/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_bringup.c stm32_spi.c)

if(CONFIG_PHOTON_DFU_BOOTLOADER)
  list(APPEND SRCS dfu_signature.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
endif()

if(CONFIG_INPUT_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
else()
  list(APPEND SRCS stm32_userleds.c)
endif()

if(CONFIG_PHOTON_WDG)
  list(APPEND SRCS stm32_wdt.c)
endif()

if(CONFIG_PHOTON_WLAN)
  list(APPEND SRCS stm32_wlan.c)
  list(APPEND SRCS stm32_wlan_firmware.c)
endif()

if(CONFIG_STM32_OTGHS)
  list(APPEND SRCS stm32_usb.c)
endif()

if(CONFIG_RGBLED)
  list(APPEND SRCS stm32_rgbled.c)
endif()

if(CONFIG_USBDEV_COMPOSITE)
  list(APPEND SRCS stm32_composite.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_PHOTON_DFU_BOOTLOADER)
  set_property(GLOBAL PROPERTY LD_SCRIPT
                               "${NUTTX_BOARD_DIR}/scripts/photon_dfu.ld")
else()
  set_property(GLOBAL PROPERTY LD_SCRIPT
                               "${NUTTX_BOARD_DIR}/scripts/photon_jtag.ld")
endif()

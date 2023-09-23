# ##############################################################################
# boards/arm/nrf53/thingy53/src/CMakeLists.txt
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

set(SRCS nrf53_boot.c nrf53_bringup.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS nrf53_appinit.c)
endif()

if(CONFIG_NRF53_APPCORE)
  list(APPEND SRCS nrf53_cpunet_boot.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS nrf53_buttons.c)
endif()

if(CONFIG_USBDEV)
  list(APPEND SRCS nrf53_usb.c)
endif()

if(CONFIG_USBDEV_COMPOSITE)
  list(APPEND SRCS nrf53_composite.c)
endif()

if(CONFIG_RGBLED)
  list(APPEND SRCS nrf53_rgbled.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_ARCH_BOARD_COMMON)
  if(CONFIG_ARCH_CHIP_NRF5340_CPUAPP)
    set_property(GLOBAL PROPERTY LD_SCRIPT
                                 "${NUTTX_COMMON_DIR}/scripts/flash_app.ld")
  else()
    set_property(GLOBAL PROPERTY LD_SCRIPT
                                 "${NUTTX_COMMON_DIR}/scripts/flash_net.ld")
  endif()
else()
  if(CONFIG_ARCH_CHIP_NRF5340_CPUAPP)
    set_property(GLOBAL PROPERTY LD_SCRIPT
                                 "${NUTTX_BOARD_DIR}/scripts/flash_app.ld")
  else()
    set_property(GLOBAL PROPERTY LD_SCRIPT
                                 "${NUTTX_BOARD_DIR}/scripts/flash_net.ld")
  endif()
endif()

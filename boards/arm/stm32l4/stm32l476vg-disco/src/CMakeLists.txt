# ##############################################################################
# boards/arm/stm32l4/stm32l476vg-disco/src/CMakeLists.txt
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

set(SRCS stm32_boot.c stm32_bringup.c stm32_spi.c)

if(CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG)
  list(APPEND SRCS stm32_clockconfig.c)
endif()

if(CONFIG_STM32_OTGFS)
  list(APPEND SRCS stm32_usb.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
else()
  list(APPEND SRCS stm32_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS stm32_buttons.c)
endif()

if(CONFIG_ADC)
  list(APPEND SRCS stm32_adc.c)
endif()

if(CONFIG_BOARDCTL_IOCTL)
  list(APPEND SRCS stm32_ioctl.c)
endif()

if(CONFIG_ETC_ROMFS)
  nuttx_add_romfs(
    NAME
    etc
    MOUNTPOINT
    etc
    RCRAWS
    etc/init.d/rcS
    etc/init.d/rc.sysinit
    PATH
    ${CMAKE_CURRENT_BINARY_DIR}/etc)

  target_link_libraries(board PRIVATE romfs_etc)
endif()

if(CONFIG_BOARDCTL_UNIQUEID)
  list(APPEND SRCS stm32_uid.c)
endif()

if(NOT CONFIG_STM32_ETHMAC)
  if(CONFIG_NETDEVICES)
    list(APPEND SRCS stm32_netinit.c)
  endif()
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT
                             "${NUTTX_BOARD_DIR}/scripts/stm32l476vg-disco.ld")

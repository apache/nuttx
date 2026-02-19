# ##############################################################################
# boards/arm/samd5e5/metro-m4/src/CMakeLists.txt
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

set(SRCS sam_boot.c sam_bringup.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS sam_appinit.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS sam_autoleds.c)
else()
  list(APPEND SRCS sam_userleds.c)
endif()

if(CONFIG_USBDEV)
  list(APPEND SRCS sam_usbdev.c)
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS sam_usbmsc.c)
endif()

if(CONFIG_USBDEV_COMPOSITE)
  list(APPEND SRCS sam_composite.c)
endif()

if(CONFIG_USBHOST)
  list(APPEND SRCS sam_usbhost.c)
endif()

if(CONFIG_FS_AUTOMOUNTER)
  list(APPEND SRCS sam_automount.c)
endif()

if(CONFIG_SAMD5E5_SERCOM5_ISI2C)
  list(APPEND SRCS sam_i2c.c)
  if(CONFIG_MTD_AT24XX)
    list(APPEND SRCS sam_at24.c)
  endif()
endif()

if(CONFIG_FS_SMARTFS)
  list(APPEND SRCS sam_smartfs.c)
endif()

if(CONFIG_BQ27426)
  list(APPEND SRCS sam_bq27426.c)
endif()

if(CONFIG_DEV_GPIO)
  list(APPEND SRCS sam_gpio.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_METRO_M4_RUNFROMFLASH)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
elseif(CONFIG_METRO_M4_RUNFROMSRAM)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/sram.ld")
endif()

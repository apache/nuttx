# ##############################################################################
# boards/arm/kinetis/freedom-k28f/src/CMakeLists.txt
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

set(SRCS k28_boot.c k28_bringup.c k28_spi.c k28_i2c.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS k28_appinit.c)
endif()

# If the RGB driver is not enabled, then treat the RGB as 3 LEDs

if(NOT CONFIG_RGBLED)
  if(CONFIG_ARCH_LEDS)
    list(APPEND SRCS k28_autoleds.c)
  else()
    list(APPEND SRCS k28_userleds.c)
  endif()
endif()

if(CONFIG_KINETIS_SDHC)
  list(APPEND SRCS k28_sdhc.c)
  if(CONFIG_FS_AUTOMOUNTER)
    list(APPEND SRCS k28_automount.c)
  endif()
endif()

if(CONFIG_PWM)
  list(APPEND SRCS k28_pwm.c)
endif()

if(CONFIG_KINETIS_USBOTG)
  list(APPEND SRCS k28_usbdev.c)
endif()

if(CONFIG_KINETIS_USBHS)
  if(CONFIG_USBHOST)
    list(APPEND SRCS k28_usbhshost.c)
  endif()
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")

# ##############################################################################
# boards/arm/n32h7/n32h762iil7/src/CMakeLists.txt
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
set(SRCS n32_boot.c n32_bringup.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS n32_appinitialize.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS n32_autoleds.c)
else()
  list(APPEND SRCS n32_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS n32_buttons.c)
endif()

if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS n32_reset.c)
endif()

if(CONFIG_BOARDCTL_UNIQUEID)
  list(APPEND SRCS n32_uid.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS n32_pwm.c)
endif()

if(CONFIG_N32H7_PROGMEM)
  list(APPEND SRCS n32_progmem.c)
endif()

if(CONFIG_N32H7_SDMMC)
  list(APPEND SRCS n32_sdmmc.c)
endif()

if(CONFIG_N32H7_USBHS)
  list(APPEND SRCS n32_usb.c)
endif()

if(CONFIG_USBDEV_COMPOSITE)
  list(APPEND SRCS n32_composite.c)
endif()

if(CONFIG_MATH_CORDIC)
  list(APPEND SRCS n32_cordic.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")

# ##############################################################################
# boards/arm/kinetis/freedom-k64f/src/CMakeLists.txt
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

set(SRCS k64_boot.c k64_spi.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS k64_autoleds.c)
else()
  list(APPEND SRCS k64_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS k64_buttons.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS k64_appinit.c k64_bringup.c)
elseif(CONFIG_BOARD_LATE_INITIALIZE)
  list(APPEND SRCS k64_bringup.c)
endif()

if(CONFIG_I2C)
  list(APPEND SRCS k64_i2c.c)
endif()

if(CONFIG_KINETIS_SDHC)
  list(APPEND SRCS k64_sdhc.c)
  if(CONFIG_FS_AUTOMOUNTER)
    list(APPEND SRCS k64_automount.c)
  endif()
endif()

if(CONFIG_USBDEV)
  list(APPEND SRCS k64_usbdev.c)
endif()

if(CONFIG_USBMSC)
  list(APPEND SRCS k64_usbmsc.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS k64_pwm.c)
endif()

if(CONFIG_BOARDCTL_UNIQUEID)
  list(APPEND SRCS k64_uid.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")

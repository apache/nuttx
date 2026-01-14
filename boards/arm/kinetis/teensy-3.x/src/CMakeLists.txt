# ##############################################################################
# boards/arm/kinetis/teensy-3.x/src/CMakeLists.txt
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

set(SRCS k20_boot.c k20_spi.c k20_i2c.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS k20_autoleds.c)
else()
  list(APPEND SRCS k20_userleds.c)
endif()
if(CONFIG_KINETIS_USBOTG)
  list(APPEND SRCS k20_usbdev.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS k20_appinit.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS k20_pwm.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_ARCH_CHIP_MK20DX256VLH7)
  set_property(GLOBAL PROPERTY LD_SCRIPT
                               "${NUTTX_BOARD_DIR}/scripts/mk20dx128vlh5.ld")
elseif(CONFIG_ARCH_CHIP_MK20DX128VLH5)
  set_property(GLOBAL PROPERTY LD_SCRIPT
                               "${NUTTX_BOARD_DIR}/scripts/mk20dx256vlh7.ld")
endif()

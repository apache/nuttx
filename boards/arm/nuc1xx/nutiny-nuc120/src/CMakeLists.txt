# ##############################################################################
# boards/arm/nuc1xx/nutiny-nuc120/src/CMakeLists.txt
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

set(SRCS nuc_boardinitialize.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS nuc_led.c)
endif()

if(CONFIG_NUC1XX_USBD)
  list(APPEND SRCS nuc_usbd.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS nuc_pwm.c)
endif()

if(CONFIG_SENSORS_QENCODER)
  list(APPEND SRCS nuc_qencoder.c)
endif()

if(CONFIG_WATCHDOG)
  list(APPEND SRCS nuc_watchdog.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT
                             "${NUTTX_BOARD_DIR}/scripts/nutiny-nuc120.ld")

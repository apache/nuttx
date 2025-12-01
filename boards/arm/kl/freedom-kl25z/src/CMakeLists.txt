# ##############################################################################
# boards/arm/kl/freedom-kl25z/src/CMakeLists.txt
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

set(SRCS kl_boardinitialize.c)

if(CONFIG_BOARDCTL)
  list(APPEND SRCS kl_appinit.c)
endif()

if(CONFIG_KL_TSI)
  list(APPEND SRCS kl_tsi.c)
endif()

if(CONFIG_KL_SPI0)
  list(APPEND SRCS kl_spi.c)
else()
  if(CONFIG_KL_SPI1)
    list(APPEND SRCS kl_spi.c)
  endif()
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS kl_led.c)
endif()

if(CONFIG_KL_USBOTG)
  list(APPEND SRCS kl_usb.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS kl_pwm.c)
endif()

if(CONFIG_SENSORS_QENCODER)
  list(APPEND SRCS kl_qencoder.c)
endif()

if(CONFIG_WATCHDOG)
  list(APPEND SRCS kl_watchdog.c)
endif()

if(CONFIG_SENSORS_ADXL345)
  list(APPEND SRCS kl_adxl345.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT
                             "${NUTTX_BOARD_DIR}/scripts/freedom-kl25z.ld")

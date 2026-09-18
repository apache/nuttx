# ##############################################################################
# boards/arm/nrf54l/nrf54l15-tag/src/CMakeLists.txt
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

set(SRCS nrf54l_boot.c nrf54l_bringup.c)
if(CONFIG_NRF54L_SPI2_MASTER)
  list(APPEND SRCS nrf54l_spi.c)
endif()
if(CONFIG_SYSTEM_I2CTOOL)
  list(APPEND SRCS nrf54l_i2c.c)
endif()
if(CONFIG_SENSORS)
  list(APPEND SRCS nrf54l_sensors.c)
endif()
if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS nrf54l_buttons.c)
endif()
if(CONFIG_NRF54L_SAADC)
  list(APPEND SRCS nrf54l_adc.c)
endif()
if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS nrf54l_autoleds.c)
else()
  list(APPEND SRCS nrf54l_userleds.c)
endif()
target_sources(board PRIVATE ${SRCS})
set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/rram.ld")

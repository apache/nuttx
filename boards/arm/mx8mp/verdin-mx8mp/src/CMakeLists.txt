# ##############################################################################
# boards/arm/mx8mp/verdin-mx8mp/src/CMakeLists.txt
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

set(SRCS mx8mp_boot.c mx8mp_bringup.c)

if(CONFIG_SENSORS_INA219)
  list(APPEND SRCS mx8mp_ina219.c)
endif()

if(CONFIG_MX8MP_I2C_DRIVER)
  list(APPEND SRCS mx8mp_i2cdev.c)
endif()

if(CONFIG_SPI)
  list(APPEND SRCS mx8mp_spi.c)
endif()

if(CONFIG_MX8MP_SPI_DRIVER)
  list(APPEND SRCS mx8mp_spidev.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS mx8mp_autoleds.c)
else()
  list(APPEND SRCS mx8mp_userleds.c)
endif()

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS mx8mp_buttons.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_MX8MP_RUN_FROM_ITCM)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/itcm.ld")
else()
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ddr.ld")
endif()

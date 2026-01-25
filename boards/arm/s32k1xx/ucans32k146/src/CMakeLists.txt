# ##############################################################################
# boards/arm/s32k1xx/ucans32k146/src/CMakeLists.txt
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

set(SRCS s32k1xx_boot.c s32k1xx_bringup.c s32k1xx_clockconfig.c
         s32k1xx_periphclocks.c)

if(CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS s32k1xx_buttons.c)
endif()

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS s32k1xx_autoleds.c)
else()
  list(APPEND SRCS s32k1xx_userleds.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS s32k1xx_appinit.c)
endif()

if(CONFIG_I2C_DRIVER)
  list(APPEND SRCS s32k1xx_i2c.c)
endif()

if(CONFIG_S32K1XX_LPSPI)
  list(APPEND SRCS s32k1xx_spi.c)
endif()

if(CONFIG_DEV_SE05X)
  list(APPEND SRCS s32k1xx_se05x.c)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_BOOT_RUNFROMFLASH)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/flash.ld")
elseif(CONFIG_BOOT_RUNFROMISRAM)
  set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/sram.ld")
endif()

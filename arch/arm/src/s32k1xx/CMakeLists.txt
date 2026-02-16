# ##############################################################################
# arch/arm/src/s32k1xx/CMakeLists.txt
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

# Source files common to all S32K1xx chip families.

list(
  APPEND
  SRCS
  s32k1xx_start.c
  s32k1xx_lowputc.c
  s32k1xx_clockconfig.c
  s32k1xx_periphclocks.c
  s32k1xx_pin.c
  s32k1xx_pingpio.c
  s32k1xx_idle.c
  s32k1xx_uid.c)

if(CONFIG_BOOT_RUNFROMFLASH)
  list(APPEND SRCS s32k1xx_flashcfg.c)
endif()

if(CONFIG_PWM)
  list(APPEND SRCS s32k1xx_pwm.c)
endif()

if(CONFIG_S32K1XX_LPUART)
  list(APPEND SRCS s32k1xx_serial.c)
endif()

if(CONFIG_S32K1XX_GPIOIRQ)
  list(APPEND SRCS s32k1xx_pinirq.c)
endif()

if(CONFIG_S32K1XX_EDMA)
  list(APPEND SRCS s32k1xx_pindma.c s32k1xx_edma.c)
endif()

if(CONFIG_DEBUG_GPIO_INFO)
  list(APPEND SRCS s32k1xx_pindump.c)
endif()

if(CONFIG_S32K1XX_LPI2C)
  list(APPEND SRCS s32k1xx_lpi2c.c s32k1xx_lpi2c_slave.c)
endif()

if(CONFIG_S32K1XX_FLEXIO_I2C)
  list(APPEND SRCS s32k1xx_flexio_i2c.c)
endif()

if(CONFIG_S32K1XX_LPSPI)
  list(APPEND SRCS s32k1xx_lpspi.c)
endif()

if(CONFIG_S32K1XX_ENET)
  list(APPEND SRCS s32k1xx_enet.c)
endif()

if(CONFIG_S32K1XX_FLEXCAN)
  list(APPEND SRCS s32k1xx_flexcan.c)
endif()

if(CONFIG_S32K1XX_RTC)
  list(APPEND SRCS s32k1xx_rtc.c)
endif()

if(CONFIG_S32K1XX_PROGMEM)
  list(APPEND SRCS s32k1xx_progmem.c)
endif()

if(CONFIG_S32K1XX_EEEPROM)
  list(APPEND SRCS s32k1xx_eeeprom.c)
endif()

if(NOT CONFIG_ARCH_CUSTOM_PMINIT)
  list(APPEND SRCS s32k1xx_pminitialize.c)
endif()

if(CONFIG_S32K1XX_RESETCAUSE_PROCFS)
  list(APPEND SRCS s32k1xx_resetcause_procfs.c)
endif()

# Source files specific to the ARM CPU family and to the S32K1xx chip family
if(CONFIG_ARCH_CHIP_S32K11X)
  add_subdirectory(s32k11x)
else()
  add_subdirectory(s32k14x)
endif()

target_sources(arch PRIVATE ${SRCS})

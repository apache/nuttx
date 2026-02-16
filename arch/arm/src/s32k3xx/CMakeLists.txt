# ##############################################################################
# arch/arm/src/s32k3xx/CMakeLists.txt
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

# Required S32K3 files

list(
  APPEND
  SRCS
  startup.S
  s32k3xx_irq.c
  s32k3xx_clrpend.c
  s32k3xx_flashboot.c
  s32k3xx_start.c
  s32k3xx_lowputc.c
  s32k3xx_clockconfig.c
  s32k3xx_periphclocks.c
  s32k3xx_pin.c
  s32k3xx_pingpio.c
  s32k3xx_idle.c
  s32k3xx_allocateheap.c)

# Configuration-dependent S32K3 files

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS s32k3xx_timerisr.c)
endif()

if(CONFIG_S32K3XX_LPUART)
  list(APPEND SRCS s32k3xx_serial.c)
endif()

if(CONFIG_S32K3XX_GPIOIRQ)
  list(APPEND SRCS s32k3xx_pinirq.c)
endif()

if(CONFIG_S32K3XX_EDMA)
  list(APPEND SRCS s32k3xx_pindma.c)
  list(APPEND SRCS s32k3xx_edma.c)
endif()

if(CONFIG_S32K3XX_LPI2C)
  list(APPEND SRCS s32k3xx_lpi2c.c)
endif()

if(CONFIG_S32K3XX_LPSPI)
  list(APPEND SRCS s32k3xx_lpspi.c)
endif()

if(CONFIG_S32K3XX_ENET)
  list(APPEND SRCS s32k3xx_emac.c)
endif()

if(CONFIG_S32K3XX_FLEXCAN)
  list(APPEND SRCS s32k3xx_flexcan.c)
endif()

if(CONFIG_S32K3XX_FS26)
  list(APPEND SRCS s32k3xx_fs26.c)
endif()

if(CONFIG_S32K3XX_QSPI)
  list(APPEND SRCS s32k3xx_qspi.c)
endif()

if(CONFIG_S32K3XX_PROGMEM)
  list(APPEND SRCS s32k3xx_progmem.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS s32k3xx_userspace.c)
endif()

if(CONFIG_ARM_MPU)
  list(APPEND SRCS s32k3xx_mpuinit.c)
endif()

target_sources(arch PRIVATE ${SRCS})

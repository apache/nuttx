# ##############################################################################
# arch/arm/src/sam34/CMakeLists.txt
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

# Required SAM34 files

list(
  APPEND
  SRCS
  sam_allocateheap.c
  sam_irq.c
  sam_lowputc.c
  sam_serial.c
  sam_start.c)

if(CONFIG_ARCH_HAVE_CUSTOM_VECTORS)
  list(APPEND SRCS sam_vectors.c)
endif()

# Configuration-dependent SAM3/4 files

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS sam_timerisr.c)
endif()

if(CONFIG_CRYPTO_AES)
  list(APPEND SRCS sam_aes.c)
endif()

if(CONFIG_ARCH_CHIP_SAM4CM)
  list(APPEND SRCS sam4cm_supc.c)
endif()

if(CONFIG_ARCH_CHIP_SAM4L)
  list(APPEND SRCS sam4l_clockconfig.c sam4l_periphclks.c sam4l_gpio.c)
else()
  list(APPEND SRCS sam_clockconfig.c sam_gpio.c sam_gpioirq.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS sam_userspace.c sam_mpuinit.c)
endif()

if(CONFIG_SAM34_CMCC)
  list(APPEND SRCS sam_cmcc.c)
endif()

if(CONFIG_SAM34_DMAC0)
  list(APPEND SRCS sam_dmac.c)
endif()

if(CONFIG_ARCH_CHIP_SAM4L)
  if(CONFIG_SAM34_PDCA)
    list(APPEND SRCS sam4l_pdca.c)
  endif()
endif()

if(CONFIG_SAM34_EMAC)
  list(APPEND SRCS sam_emac.c)
endif()

if(CONFIG_SAM34_UDP)
  list(APPEND SRCS sam_udp.c)
endif()

if(CONFIG_SAM34_HSMCI)
  list(APPEND SRCS sam_hsmci.c)
endif()

if(CONFIG_SAM34_EXTNAND)
  list(APPEND SRCS sam4s_nand.c)
endif()

if(CONFIG_SAM34_SPI0)
  list(APPEND SRCS sam_spi.c)
else()
  if(CONFIG_SAM34_SPI1)
    list(APPEND SRCS sam_spi.c)
  endif()
endif()

if(CONFIG_SAM34_TWIM)
  list(APPEND SRCS sam_twi.c)
endif()

if(CONFIG_SAM34_AES)
  list(APPEND SRCS sam_aes.c)
endif()

if(CONFIG_CRYPTO_CRYPTODEV_HARDWARE)
  list(APPEND SRCS sam_crypto.c)
endif()

if(CONFIG_SAM34_RTC)
  list(APPEND SRCS sam_rtc.c)
endif()

if(CONFIG_SAM34_RTT)
  list(APPEND SRCS sam_rtt.c)
endif()

if(CONFIG_SAM34_WDT)
  list(APPEND SRCS sam_wdt.c)
endif()

if(CONFIG_TIMER)
  list(APPEND SRCS sam_tc.c)
endif()

if(CONFIG_ARCH_CHIP_SAM4CM)
  if(CONFIG_SAM34_TC)
    list(APPEND SRCS sam4cm_tc.c)

    if(CONFIG_SAM34_ONESHOT)
      list(APPEND SRCS sam4cm_oneshot.c sam4cm_oneshot_lowerhalf.c)
    endif()

    if(CONFIG_SAM34_FREERUN)
      list(APPEND SRCS sam4cm_freerun.c)
    endif()

    if(CONFIG_SCHED_TICKLESS)
      list(APPEND SRCS sam4cm_tickless.c)
    endif()
  endif()

  if(CONFIG_SMP)
    list(APPEND SRCS sam4cm_cpuidlestack.c)
    list(APPEND SRCS sam4cm_smpcall.c sam4cm_cpustart.c)
    if(NOT CONFIG_ARCH_IDLE_CUSTOM)
      list(APPEND SRCS sam4cm_idle.c)
    endif()
  endif()

  if(CONFIG_ARCH_HAVE_MULTICPU)
    list(APPEND SRCS sam4cm_cpuindex.c)
  endif()

endif()

target_sources(arch PRIVATE ${SRCS})

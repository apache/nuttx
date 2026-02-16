# ##############################################################################
# arch/arm/src/imxrt/CMakeLists.txt
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

# Common ARM and Cortex-M7 files

# Required i.MX RT files

set(SRCS imxrt_allocateheap.c imxrt_start.c imxrt_periphclks.c)

if(CONFIG_IMXRT_PROGMEM)
  list(APPEND SRCS imxrt_flash.c)
endif()

list(
  APPEND
  SRCS
  imxrt_irq.c
  imxrt_clrpend.c
  imxrt_gpio.c
  imxrt_daisy.c
  imxrt_wdog.c
  imxrt_serial.c
  imxrt_xbar.c
  imxrt_ocotp.c
  imxrt_lowputc.c)

# Configuration-dependent i.MX RT files

if(CONFIG_IMXRT_CLOCKCONFIG_VER2)
  list(APPEND SRCS imxrt_clockconfig_ver2.c imxrt_pmu.c)
else()
  list(APPEND SRCS imxrt_clockconfig_ver1.c)
endif()

if(CONFIG_IMXRT_IOMUX_VER2)
  list(APPEND SRCS imxrt_iomuxc_ver2.c)
else()
  list(APPEND SRCS imxrt_iomuxc_ver1.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS imxrt_idle.c)
endif()

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS imxrt_timerisr.c)
else()
  list(APPEND SRCS imxrt_tickless.c)
endif()

if(CONFIG_IMXRT_GPIO_IRQ)
  list(APPEND SRCS imxrt_gpioirq.c)
endif()

if(CONFIG_ARM_MPU)
  list(APPEND SRCS imxrt_mpuinit.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS imxrt_userspace.c)
endif()

if(CONFIG_IMXRT_EDMA)
  list(APPEND SRCS imxrt_edma.c)
endif()

if(CONFIG_IMXRT_USDHC)
  list(APPEND SRCS imxrt_usdhc.c)
endif()

if(CONFIG_IMXRT_LCD)
  list(APPEND SRCS imxrt_lcd.c)
endif()

if(CONFIG_IMXRT_FLEXCAN)
  list(APPEND SRCS imxrt_flexcan.c)
endif()

if(CONFIG_IMXRT_FLEXIO)
  list(APPEND SRCS imxrt_flexio.c)
endif()

if(CONFIG_IMXRT_FLEXPWM)
  list(APPEND SRCS imxrt_flexpwm.c)
endif()

if(CONFIG_IMXRT_SNVS_LPSRTC)
  list(APPEND SRCS imxrt_lpsrtc.c imxrt_hprtc.c)
elseif(CONFIG_IMXRT_SNVS_HPRTC)
  list(APPEND SRCS imxrt_hprtc.c)
endif()

if(CONFIG_RTC_DRIVER)
  list(APPEND SRCS imxrt_rtc_lowerhalf.c)
endif()

if(CONFIG_IMXRT_ENET)
  list(APPEND SRCS imxrt_enet.c)
endif()

if(CONFIG_IMXRT_LPI2C)
  list(APPEND SRCS imxrt_lpi2c.c)
endif()

if(CONFIG_IMXRT_LPSPI)
  list(APPEND SRCS imxrt_lpspi.c)
endif()

if(CONFIG_IMXRT_FLEXSPI)
  list(APPEND SRCS imxrt_flexspi.c)
endif()

if(CONFIG_IMXRT_ENC)
  list(APPEND SRCS imxrt_enc.c)
endif()

if(CONFIG_IMXRT_USBOTG)
  list(APPEND SRCS imxrt_ehci.c)
endif()

if(CONFIG_IMXRT_USBDEV)
  list(APPEND SRCS imxrt_usbdev.c)
endif()

if(CONFIG_IMXRT_ADC)
  if(CONFIG_IMXRT_ADC_VER2)
    list(APPEND SRCS imxrt_adc_ver2.c)
  else()
    list(APPEND SRCS imxrt_adc_ver1.c)
  endif()
endif()

target_sources(arch PRIVATE ${SRCS})

# ##############################################################################
# arch/arm/src/cxd56xx/CMakeLists.txt
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

set(SRCS
    cxd56_farapistub.S
    cxd56_allocateheap.c
    cxd56_idle.c
    cxd56_uid.c
    cxd56_serial.c
    cxd56_uart.c
    cxd56_irq.c
    cxd56_start.c
    cxd56_timerisr.c
    cxd56_pinconfig.c
    cxd56_clock.c
    cxd56_delay.c
    cxd56_gpio.c
    cxd56_pmic.c
    cxd56_cpufifo.c
    cxd56_icc.c
    cxd56_powermgr.c
    cxd56_farapi.c
    cxd56_sysctl.c)

if(CONFIG_SMP)
  list(APPEND SRCS cxd56_cpuidlestack.c)
  list(APPEND SRCS cxd56_cpuindex.c)
  list(APPEND SRCS cxd56_cpupause.c)
  list(APPEND SRCS cxd56_cpustart.c)
  if(CONFIG_CXD56_TESTSET)
    list(APPEND SRCS cxd56_testset.c)
  endif()
endif()

if(CONFIG_ARCH_USE_TEXT_HEAP)
  list(APPEND SRCS cxd56_textheap.c)
endif()

if(CONFIG_CXD56_UART0)
  list(APPEND SRCS cxd56_uart0.c)
endif()

if(CONFIG_CXD56_PM_PROCFS)
  list(APPEND SRCS cxd56_powermgr_procfs.c)
endif()

if(CONFIG_CXD56_RTC)
  list(APPEND SRCS cxd56_rtc.c)
  if(CONFIG_RTC_DRIVER)
    list(APPEND SRCS cxd56_rtc_lowerhalf.c)
  endif()
endif()

if(CONFIG_CXD56_GPIO_IRQ)
  list(APPEND SRCS cxd56_gpioint.c)
endif()

if(CONFIG_USBDEV)
  list(APPEND SRCS cxd56_usbdev.c)
endif()

if(CONFIG_CXD56_SDIO)
  list(APPEND SRCS cxd56_sdhci.c)
endif()

if(CONFIG_CXD56_SFC)
  list(APPEND SRCS cxd56_sfc.c)
endif()

if(CONFIG_CXD56_SPH)
  list(APPEND SRCS cxd56_sph.c)
endif()

if(CONFIG_CXD56_EMMC)
  list(APPEND SRCS cxd56_emmc.c)
endif()

if(CONFIG_CXD56_SPI)
  list(APPEND SRCS cxd56_spi.c)
endif()

if(CONFIG_CXD56_I2C)
  list(APPEND SRCS cxd56_i2c.c)
endif()

if(CONFIG_I2C_BITBANG)
  list(APPEND SRCS cxd56_i2c_bitbang.c)
endif()

if(CONFIG_CXD56_DMAC)
  list(APPEND SRCS cxd56_dmac.c)
endif()

if(CONFIG_CXD56_PWM)
  list(APPEND SRCS cxd56_pwm.c)
endif()

if(CONFIG_CXD56_GAUGE)
  list(APPEND SRCS cxd56_gauge.c)
endif()

if(CONFIG_CXD56_CHARGER)
  list(APPEND SRCS cxd56_charger.c)
endif()

if(CONFIG_CXD56_GE2D)
  list(APPEND SRCS cxd56_ge2d.c)
endif()

if(CONFIG_CXD56_CISIF)
  list(APPEND SRCS cxd56_cisif.c)
endif()

if(CONFIG_CXD56_SCU)
  list(APPEND SRCS cxd56_scu.c cxd56_scufifo.c)
  if(CONFIG_CXD56_ADC)
    list(APPEND SRCS cxd56_adc.c)
  endif()
  if(CONFIG_CXD56_UDMAC)
    list(APPEND SRCS cxd56_udmac.c)
  endif()
endif()

if(CONFIG_CXD56_TIMER)
  list(APPEND SRCS cxd56_timer.c)
endif()

if(CONFIG_CXD56_WDT)
  list(APPEND SRCS cxd56_wdt.c)
endif()

if(CONFIG_CXD56_GNSS)
  list(APPEND SRCS cxd56_gnss.c)
  list(APPEND SRCS cxd56_cpu1signal.c)
endif()

if(CONFIG_CXD56_GEOFENCE)
  list(APPEND SRCS cxd56_geofence.c)
endif()

if(CONFIG_CXD56_BACKUPLOG)
  list(APPEND SRCS cxd56_backuplog.c)
endif()

if(CONFIG_CXD56_HOSTIF)
  list(APPEND SRCS cxd56_hostif.c)
endif()

target_sources(arch PRIVATE ${SRCS})

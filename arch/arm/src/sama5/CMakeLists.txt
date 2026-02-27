# ##############################################################################
# arch/arm/src/sama5/CMakeLists.txt
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

# SAMA5-specific C source files

list(
  APPEND
  SRCS
  sam_allocateheap.c
  sam_boot.c
  sam_clockconfig.c
  sam_irq.c
  sam_lowputc.c
  sam_memories.c
  sam_memorymap.c
  sam_pck.c
  sam_pio.c
  sam_pmc.c
  sam_sckc.c
  sam_serial.c
  sam_serialinit.c)

# Configuration dependent C and assembly language files

if(CONFIG_MM_PGALLOC)
  list(APPEND SRCS sam_pgalloc.c)
endif()

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS sam_timerisr.c)
endif()

if(CONFIG_SAMA5_SYSTEMRESET)
  list(APPEND SRCS sam_systemreset.c)
endif()

if(CONFIG_SAMA5_DMAC0)
  list(APPEND SRCS sam_dmac.c)
else()
  if(CONFIG_SAMA5_DMAC1)
    list(APPEND SRCS sam_dmac.c)
  endif()
endif()

if(CONFIG_SAMA5_XDMAC0)
  list(APPEND SRCS sam_xdmac.c)
else()
  if(CONFIG_SAMA5_XDMAC1)
    list(APPEND SRCS sam_xdmac.c)
  endif()
endif()

if(CONFIG_SAMA5_PIO_IRQ)
  list(APPEND SRCS sam_pioirq.c)
endif()

if(CONFIG_SAMA5_RTC)
  list(APPEND SRCS sam_rtc.c)
endif()

if(CONFIG_SAMA5_WDT)
  list(APPEND SRCS sam_wdt.c)
endif()

if(CONFIG_SAMA5_DBGU)
  list(APPEND SRCS sam_dbgu.c)
endif()

if(CONFIG_SAMA5_TRNG)
  list(APPEND SRCS sam_trng.c)
endif()

if(CONFIG_SAMA5_FLEXCOM_USART)
  list(APPEND SRCS sam_flexcom_serial.c)
endif()

if(CONFIG_SAMA5_FLEXCOM_SPI)
  list(APPEND SRCS sam_flexcom_spi.c)
endif()

if(CONFIG_SAMA5_SPI0)
  list(APPEND SRCS sam_spi.c)
else()
  if(CONFIG_SAMA5_SPI1)
    list(APPEND SRCS sam_spi.c)
  endif()
endif()

if(CONFIG_SAMA5_QSPI0)
  list(APPEND SRCS sam_qspi.c)
else()
  if(CONFIG_SAMA5_QSPI1)
    list(APPEND SRCS sam_qspi.c)
  endif()
endif()

if(CONFIG_SAMA5D2_CLASSD)
  list(APPEND SRCS sam_classd.c)
endif()

if(CONFIG_SAMA5_SSC0)
  list(APPEND SRCS sam_ssc.c)
else()
  if(CONFIG_SAMA5_SSC1)
    list(APPEND SRCS sam_ssc.c)
  endif()
endif()

if(CONFIG_SAMA5_LCDC)
  list(APPEND SRCS sam_lcd.c)
endif()

if(CONFIG_SAMA5_UHPHS)
  if(CONFIG_SAMA5_OHCI)
    list(APPEND SRCS sam_ohci.c)
  endif()
  if(CONFIG_SAMA5_EHCI)
    list(APPEND SRCS sam_ehci.c)
  endif()
endif()

if(CONFIG_SAMA5_UDPHS)
  list(APPEND SRCS sam_udphs.c)
endif()

if(CONFIG_USBHOST)
  if(CONFIG_USBHOST_TRACE)
    list(APPEND SRCS sam_usbhost.c)
  else()
    if(CONFIG_DEBUG_USB)
      list(APPEND SRCS sam_usbhost.c)
    endif()
  endif()
endif()

if(CONFIG_SAMA5_SDMMC0)
  list(APPEND SRCS sam_sdmmc.c)
else()
  if(CONFIG_SAMA5_SDMMC1)
    list(APPEND SRCS sam_sdmmc.c)
  endif()
endif()

if(CONFIG_SAMA5_HSMCI0)
  list(APPEND SRCS sam_hsmci.c sam_hsmci_clkdiv.c)
else()
  if(CONFIG_SAMA5_HSMCI1)
    list(APPEND SRCS sam_hsmci.c sam_hsmci_clkdiv.c)
  else()
    if(CONFIG_SAMA5_HSMCI2)
      list(APPEND SRCS sam_hsmci.c sam_hsmci_clkdiv.c)
    endif()
  endif()
endif()

if(CONFIG_NET)
  list(APPEND SRCS sam_ethernet.c)
  if(CONFIG_SAMA5_EMACA)
    list(APPEND SRCS sam_emaca.c)
  endif()
  if(CONFIG_SAMA5_EMACB)
    list(APPEND SRCS sam_emacb.c)
  endif()
  if(CONFIG_SAMA5_GMAC)
    list(APPEND SRCS sam_gmac.c)
  endif()
endif()

if(CONFIG_SAMA5_CAN0)
  list(APPEND SRCS sam_can.c)
else()
  if(CONFIG_SAMA5_CAN1)
    list(APPEND SRCS sam_can.c)
  endif()
endif()

if(CONFIG_SAMA5_MCAN0)
  list(APPEND SRCS sam_mcan.c)
else()
  if(CONFIG_SAMA5_MCAN1)
    list(APPEND SRCS sam_mcan.c)
  endif()
endif()

if(CONFIG_SAMA5_TWI0)
  list(APPEND SRCS sam_twi.c)
else()
  if(CONFIG_SAMA5_TWI1)
    list(APPEND SRCS sam_twi.c)
  else()
    if(CONFIG_SAMA5_TWI2)
      list(APPEND SRCS sam_twi.c)
    endif()
    if(CONFIG_SAMA5_FLEXCOM_TWI)
      list(APPEND SRCS sam_twi.c)
    endif()
  endif()
endif()

if(CONFIG_SAMA5_ADC)
  list(APPEND SRCS sam_adc.c)
  if(CONFIG_SAMA5_TSD)
    list(APPEND SRCS sam_tsd.c)
  endif()
endif()

if(CONFIG_SAMA5_PWM)
  list(APPEND SRCS sam_pwm.c)
endif()

if(CONFIG_SAMA5_HAVE_TC)
  list(APPEND SRCS sam_tc.c)
  if(CONFIG_SAMA5_ONESHOT)
    list(APPEND SRCS sam_oneshot.c sam_oneshot_lowerhalf.c)
  endif()
  if(CONFIG_SAMA5_FREERUN)
    list(APPEND SRCS sam_freerun.c)
  endif()
  if(CONFIG_SCHED_TICKLESS)
    list(APPEND SRCS sam_tickless.c)
  endif()
endif()

if(CONFIG_SAMA5_SFC)
  list(APPEND SRCS sam_sfc.c)
endif()

if(CONFIG_SAMA5_EBICS0_NAND)
  list(APPEND SRCS sam_nand.c sam_pmecc.c sam_gf512.c sam_gf1024.c)
else()
  if(CONFIG_SAMA5_EBICS1_NAND)
    list(APPEND SRCS sam_nand.c sam_pmecc.c sam_gf512.c sam_gf1024.c)
  else()
    if(CONFIG_SAMA5_EBICS2_NAND)
      list(APPEND SRCS sam_nand.c sam_pmecc.c sam_gf512.c sam_gf1024.c)
    else()
      if(CONFIG_SAMA5_EBICS3_NAND)
        list(APPEND SRCS sam_nand.c sam_pmecc.c sam_gf512.c sam_gf1024.c)
      endif()
    endif()
  endif()
endif()

target_sources(arch PRIVATE ${SRCS})

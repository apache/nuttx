############################################################################
# boards/arm/stm32/stm32f4discovery/src/Make.defs
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

set(SRCS stm32_boot.c stm32_bringup.c stm32_spi.c)

if (CONFIG_ARCH_LEDS)
  list(APPEND SRCS stm32_autoleds.c)
else()
  list(APPEND SRCS stm32_userleds.c)
endif()

if (CONFIG_SCHED_CRITMONITOR)
  list(APPEND SRCS stm32_critmon.c)
endif()

if (CONFIG_AUDIO_CS43L22)
  list(APPEND SRCS stm32_cs43l22.c)
endif()

if (CONFIG_ARCH_BUTTONS)
  list(APPEND SRCS  stm32_buttons.c)
endif()

if (CONFIG_CAN)
  list(APPEND SRCS stm32_can.c)
endif()

if (CONFIG_STM32_OTGFS)
  list(APPEND SRCS stm32_usb.c)
endif()

if (CONFIG_LCD_ST7567)
  list(APPEND SRCS stm32_st7567.c)
endif()

if (CONFIG_ENC28J60)
  list(APPEND SRCS stm32_enc28j60.c)
endif()

if (CONFIG_LPWAN_SX127X)
  list(APPEND SRCS stm32_sx127x.c)
endif()

if (CONFIG_LCD_MAX7219)
  list(APPEND SRCS stm32_max7219.c)
endif()

if (CONFIG_LCD_ST7032)
  list(APPEND SRCS stm32_st7032.c)
endif()

if (CONFIG_PCA9635PW)
  list(APPEND SRCS stm32_pca9635.c)
endif()

if (CONFIG_STM32_SDIO)
  list(APPEND SRCS stm32_sdio.c)
endif()

if (CONFIG_STM32_ETHMAC)
  list(APPEND SRCS stm32_ethernet.c)
endif()

if (CONFIG_LEDS_MAX7219)
  list(APPEND SRCS stm32_max7219_leds.c)
endif()

if (CONFIG_RGBLED)
  list(APPEND SRCS stm32_rgbled.c)
endif()

if (CONFIG_RTC_DS1307)
  list(APPEND SRCS stm32_ds1307.c)
endif()

if (CONFIG_PWM)
  list(APPEND SRCS stm32_pwm.c)
endif()

if (CONFIG_LIB_BOARDCTL)
  list(APPEND SRCS stm32_appinit.c)
  if (CONFIG_BOARDCTL_RESET)
    list(APPEND SRCS stm32_reset.c)
  endif()
endif()

if (CONFIG_ARCH_CUSTOM_PMINIT)
  list(APPEND SRCS stm32_pm.c)
endif()

if (CONFIG_PM_BUTTONS)
  list(APPEND SRCS stm32_pmbuttons.c)
endif()

if (CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS stm32_idle.c)
endif()

if (CONFIG_STM32_FSMC)
  list(APPEND SRCS stm32_extmem.c)

  if (CONFIG_LCD_SSD1289)
    list(APPEND SRCS stm32_ssd1289.c)
  endif()
endif()

if (CONFIG_LCD_SSD1351)
  list(APPEND SRCS stm32_ssd1351.c)
endif()

if (CONFIG_LCD_UG2864AMBAG01)
  list(APPEND SRCS stm32_ug2864ambag01.c)
endif()

if (CONFIG_LCD_UG2864HSWEG01)
  list(APPEND SRCS stm32_ug2864hsweg01.c)
endif()

if (CONFIG_TESTING_OSTEST)
  list(APPEND SRCS stm32_ostest.c)
endif()

if (CONFIG_TIMER)
  list(APPEND SRCS stm32_timer.c)
endif()

if (CONFIG_STM32_HCIUART)
  if (CONFIG_BLUETOOTH_UART)
    list(APPEND SRCS stm32_hciuart.c)
  endif()
endif()

if (CONFIG_STM32_ROMFS)
  list(APPEND SRCS stm32_romfs_initialize.c)
endif()

if (CONFIG_BOARDCTL_UNIQUEID)
  list(APPEND SRCS stm32_uid.c)
endif()

if (CONFIG_USBMSC)
  list(APPEND SRCS stm32_usbmsc.c)
endif()

if (NOT CONFIG_STM32_ETHMAC)
  if (CONFIG_NETDEVICES)
    list(APPEND SRCS stm32_netinit.c)
  endif()
endif()

if (CONFIG_MMCSD_SPI)
  list(APPEND SRCS stm32_mmcsd.c)
endif()

if (CONFIG_WL_GS2200M)
  list(APPEND SRCS stm32_gs2200m.c)
endif()

if (CONFIG_LCD_ST7789)
  list(APPEND SRCS stm32_st7789.c)
endif()

target_sources(board PRIVATE ${SRCS})

# TODO: make this the default and then allow boards to redefine
set_property(GLOBAL PROPERTY LD_SCRIPT "${BOARD_PATH}/scripts/ld.script")

# TODO:move this to apropriate arch/toolchain level
set_property(GLOBAL APPEND PROPERTY COMPILE_OPTIONS $<$<CONFIG:Release>:-fno-strict-aliasing -fomit-frame-pointer>)

# TODO: see where to put pic flags
#set_property(TARGET nuttx APPEND PROPERTY NUTTX_COMPILE_OPTIONS $<$<NOT:$<COMPILE_LANGUAGE:ASM>>:-fpic -msingle-pic-base -mpic-register=r10>)

#ifeq ($(CONFIG_ARMV7M_TOOLCHAIN_CLANGL),y)
#  ARCHCFLAGS += -nostdlib -ffreestanding
#  ARCHCXXFLAGS += -nostdlib -ffreestanding
#else
#  ARCHCFLAGS += -funwind-tables
#  ARCHCXXFLAGS += -fno-rtti -funwind-tables
#  ifneq ($(CONFIG_DEBUG_NOOPT),y)
#    ARCHOPTIMIZATION += -fno-strength-reduce
#  endif
#endif

set_property(TARGET nuttx APPEND PROPERTY NUTTX_COMPILE_OPTIONS -funwind-tables)
set_property(GLOBAL APPEND PROPERTY COMPILE_OPTIONS -fno-strength-reduce)

# TODO: nxflat
#NXFLATLDFLAGS1 = -r -d -warn-common
#NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-pcrel.ld -no-check-sections
#LDNXFLATFLAGS = -e main -s 2048

# Loadable module definitions

set_property(TARGET nuttx APPEND PROPERTY NUTTX_ELF_MODULE_COMPILE_OPTIONS -mlong-calls)
set_property(TARGET nuttx APPEND PROPERTY NUTTX_ELF_MODULE_LINK_OPTIONS -r -e module_initialize  -T ${NUTTX_DIR}/libs/libc/modlib/gnu-elf.ld)

# ELF module definitions

set_property(TARGET nuttx APPEND PROPERTY NUTTX_ELF_APP_COMPILE_OPTIONS -mlong-calls)
set_property(TARGET nuttx APPEND PROPERTY NUTTX_ELF_APP_LINK_OPTIONS -r -e main -T${BOARD_PATH}/scripts/gnu-elf.ld)

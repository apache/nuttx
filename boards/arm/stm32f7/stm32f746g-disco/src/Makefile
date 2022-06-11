############################################################################
# boards/arm/stm32f7/stm32f746g-disco/src/Makefile
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

include $(TOPDIR)/Make.defs

CSRCS = stm32_boot.c stm32_spi.c stm32_bringup.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += stm32_autoleds.c
else
CSRCS += stm32_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS +=  stm32_buttons.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += stm32_appinitialize.c
endif

ifeq ($(CONFIG_ADC),y)
CSRCS += stm32_adc.c
endif

ifeq ($(CONFIG_SPORADIC_INSTRUMENTATION),y)
CSRCS += stm32_sporadic.c
endif

ifeq ($(CONFIG_STM32F7_LTDC),y)
CSRCS += stm32_lcd.c
endif

ifeq ($(CONFIG_STM32F7_FMC),y)
CSRCS += stm32_extmem.c
endif

ifeq ($(CONFIG_INPUT_FT5X06),y)
CSRCS += stm32_touchscreen.c
endif

ifeq ($(CONFIG_MTD_N25QXXX),y)
CSRCS += stm32_n25q.c
endif

ifeq ($(CONFIG_STM32F7_SDMMC),y)
CSRCS += stm32_sdmmc.c
endif

ifeq ($(CONFIG_AUDIO_WM8994),y)
CSRCS += stm32_wm8994.c
endif

include $(TOPDIR)/boards/Board.mk

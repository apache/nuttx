############################################################################
# boards/arm/stm32l4/stm32l4r9ai-disco/src/Makefile
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

CSRCS = stm32_boot.c

ifeq ($(CONFIG_ARCH_BOARD_STM32L4_CUSTOM_CLOCKCONFIG),y)
CSRCS += stm32_clockconfig.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += stm32_autoleds.c
else
CSRCS += stm32_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += stm32_buttons.c
endif

ifeq ($(CONFIG_ADC),y)
ifeq ($(CONFIG_STM32L4_ADC),y)
CSRCS += stm32_adc.c
endif
ifeq ($(CONFIG_STM32L4_DFSDM),y)
CSRCS += stm32_dfsdm.c
endif
endif

ifeq ($(CONFIG_DAC),y)
CSRCS += stm32_dac.c
endif

ifeq ($(CONFIG_STM32L4_SPI),y)
CSRCS += stm32_spi.c
endif

ifeq ($(CONFIG_STM32L4_OTGFS),y)
CSRCS += stm32_usb.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += stm32_appinit.c
endif

include $(TOPDIR)/boards/Board.mk

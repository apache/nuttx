############################################################################
# boards/arm/lpc17xx_40xx/zkit-arm-1769/src/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/Make.defs

CSRCS = lpc17_40_boot.c lpc17_40_leds.c lpc17_40_ssp.c lpc17_40_buttons.c lpc17_40_lcd.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += lpc17_40_appinit.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += lpc17_40_usbmsc.c
endif

ifeq ($(CONFIG_ADC),y)
CSRCS += lpc17_40_adc.c
endif

ifeq ($(CONFIG_DAC),y)
CSRCS += lpc17_40_dac.c
endif

ifeq ($(CONFIG_CAN),y)
CSRCS += lpc17_40_can.c
endif

ifeq ($(CONFIG_LPC17_40_SPI),y)
CSRCS += lpc17_40_spi.c
endif

include $(TOPDIR)/boards/Board.mk

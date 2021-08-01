############################################################################
# boards/arm/lpc17xx_40xx/open1788/src/Makefile
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

CSRCS = lpc17_40_boardinitialize.c lpc17_40_bringup.c lpc17_40_ssp.c

ifeq ($(CONFIG_LPC17_40_EMC),y)
ifeq ($(CONFIG_LPC17_40_EXTNOR),y)
  CSRCS += lpc17_40_norinitialize.c
endif

ifeq ($(CONFIG_LPC17_40_EXTNAND),y)
  CSRCS += lpc17_40_nandinitialize.c
endif

ifeq ($(CONFIG_LPC17_40_EXTDRAM),y)
  CSRCS += lpc17_40_sdraminitialize.c
endif
endif

ifeq ($(CONFIG_LPC17_40_LCD),y)
  CSRCS += lpc17_40_lcd.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
  CSRCS += lpc17_40_appinit.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
  CSRCS += lpc17_40_autoleds.c
else
  CSRCS += lpc17_40_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
  CSRCS += lpc17_40_buttons.c
else ifeq ($(CONFIG_OPEN1788_DJOYSTICK),y)
  CSRCS += lpc17_40_djoystick.c
endif

ifeq ($(CONFIG_INPUT_ADS7843E),y)
  CSRCS += lpc17_40_touchscreen.c
endif

include $(TOPDIR)/boards/Board.mk

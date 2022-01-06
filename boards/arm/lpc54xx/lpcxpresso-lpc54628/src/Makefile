############################################################################
# boards/arm/lpc54xx/lpcxpresso-lpc54628/src/Makefile
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

CSRCS = lpc54_boot.c lpc54_bringup.c lpc54_userleds.c lpc54_lcd.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += lpc54_autoleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += lpc54_buttons.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += lpc54_appinit.c
endif

ifeq ($(CONFIG_LPC54_HAVE_I2C_MASTER),y)
CSRCS += lpc54_i2c.c
endif

ifeq ($(CONFIG_SYSTEM_I2CTOOL),y)
CSRCS += lpc54_i2ctool.c
endif

ifeq ($(CONFIG_INPUT_FT5X06),y)
CSRCS += lpc54_ft5x06.c
endif

ifeq ($(CONFIG_LPC54_EMC),y)
ifeq ($(CONFIG_LPC54_EMC_DYNAMIC),y)
CSRCS += lpc54_sdram.c
endif
endif

include $(TOPDIR)/boards/Board.mk

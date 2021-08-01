############################################################################
# boards/arm/sam34/sam4e-ek/src/Makefile
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

CSRCS = sam_boot.c sam_leds.c sam_buttons.c sam_udp.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += sam_appinit.c
endif

ifeq ($(CONFIG_SAM34_HSMCI),y)
CSRCS += sam_hsmci.c
endif

ifeq ($(CONFIG_LCD),y)
ifeq ($(CONFIG_SAM4EEK_LCD_ILI9325),y)
CSRCS += sam_ili9325.c
else
ifeq ($(CONFIG_SAM4EEK_LCD_ILI9341),y)
CSRCS += sam_ili9341.c
endif
endif
endif

ifeq ($(CONFIG_SAM34_EMAC),y)
CSRCS += sam_ethernet.c
endif

ifeq ($(CONFIG_INPUT_ADS7843E),y)
CSRCS += sam_ads7843e.c
endif

ifeq ($(CONFIG_SAM34_SPI0),y)
CSRCS += sam_spi.c
ifeq ($(CONFIG_MTD_AT25),y)
CSRCS += sam_at25.c
endif
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += sam_usbmsc.c
endif

include $(TOPDIR)/boards/Board.mk

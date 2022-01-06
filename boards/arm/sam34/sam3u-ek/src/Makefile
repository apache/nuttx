############################################################################
# boards/arm/sam34/sam3u-ek/src/Makefile
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

CSRCS = sam_boot.c sam_leds.c sam_buttons.c sam_spi.c sam_usbdev.c

ifeq ($(CONFIG_LCD),y)
CSRCS += sam_lcd.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += sam_appinit.c
endif

ifeq ($(CONFIG_SAM34_HSMCI),y)
CSRCS += sam_mmcsd.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += sam_usbmsc.c
endif

ifeq ($(CONFIG_INPUT_ADS7843E),y)
CSRCS += sam_touchscreen.c
endif

include $(TOPDIR)/boards/Board.mk

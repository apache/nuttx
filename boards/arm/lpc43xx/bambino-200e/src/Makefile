############################################################################
# boards/arm/lpc43xx/bambino-200e/src/Makefile
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

CSRCS = lpc43_boot.c lpc43_ssp.c

ifeq ($(CONFIG_SENSORS_MAX31855),y)
CSRCS += lpc43_max31855.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += lpc43_appinit.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += lpc43_autoleds.c
else
CSRCS += lpc43_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += lpc43_buttons.c
endif

ifeq ($(CONFIG_TIMER),y)
CSRCS += lpc43_timer.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += lpc43_usbmsc.c
endif

ifeq ($(CONFIG_SPIFI_LIBRARY),y)
CFLAGS += -DEUROBIRD
SPIFI_LIB = spifi_lib
CSRCS += $(SPIFI_LIB)/spifi_rom_api.c
#CSRCS += $(SPIFI_LIB)/amic.c
#CSRCS += $(SPIFI_LIB)/atmel.c
#CSRCS += $(SPIFI_LIB)/chi.c
#CSRCS += $(SPIFI_LIB)/eon.c
#CSRCS += $(SPIFI_LIB)/esmt.c
#CSRCS += $(SPIFI_LIB)/esmt.c
#CSRCS += $(SPIFI_LIB)/giga.c
#CSRCS += $(SPIFI_LIB)/macronix.c
#CSRCS += $(SPIFI_LIB)/numonyx.c
CSRCS += $(SPIFI_LIB)/spansion.c
#CSRCS += $(SPIFI_LIB)/sst.c
CSRCS += $(SPIFI_LIB)/winbond.c
endif

include $(TOPDIR)/boards/Board.mk

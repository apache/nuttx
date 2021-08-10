############################################################################
# boards/arm/lpc31xx/olimex-lpc-h3131/src/Makefile
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

CSRCS = lpc31_boot.c lpc31_clkinit.c

ifeq ($(CONFIG_LPC31_EXTDRAM),y)
CSRCS += lpc31_mem.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += lpc31_leds.c
endif

ifeq ($(CONFIG_LPC31_SPI),y)
CSRCS += lpc31_spi.c
endif

ifeq ($(CONFIG_LPC31_MCI),y)
CSRCS += lpc31_mmcsd.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += lpc31_appinit.c
endif

ifeq ($(CONFIG_LPC31_USBOTG),y)
ifeq ($(CONFIG_USBHOST),y)
CSRCS += lpc31_usbhost.c
endif
endif

include $(TOPDIR)/boards/Board.mk

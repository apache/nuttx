##############################################################################
# boards/arm/lpc17xx_40xx/pnev5180b/src/Makefile
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
##############################################################################

include $(TOPDIR)/Make.defs

CSRCS = lpc17_40_boot.c lpc17_40_bringup.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += lpc17_40_leds.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += lpc17_40_appinit.c
endif

ifeq ($(CONFIG_USBDEV_COMPOSITE),y)
CSRCS += lpc17_40_composite.c
endif

ifeq ($(CONFIG_LPC17_40_SPI),y)
CSRCS += lpc17_40_spi.c
endif

ifeq ($(CONFIG_ELF),y)
CSRCS += lpc17_40_symtab.c
endif

ifeq ($(CONFIG_NXFLAT),y)
CSRCS += lpc17_40_symtab.c
endif

ifeq ($(CONFIG_LPC17_40_ROMFS),y)
CSRCS += lpc17_40_romfs.c
endif

include $(TOPDIR)/boards/Board.mk

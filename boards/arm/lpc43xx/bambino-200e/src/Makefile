############################################################################
# boards/arm/lpc43xx/bambino-200e/src/Makefile
#
#   Copyright (C) 2012 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#           Alan Carvalho de Assis acassis@gmail.com [nuttx] <nuttx@googlegroups.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

include $(TOPDIR)/Make.defs

CSRCS = lpc43_boot.c lpc43_ssp.c

ifeq ($(CONFIG_SENSORS_MAX31855),y)
CSRCS += lpc43_max31855.c
endif

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += lpc43_appinit.c
endif

ifeq ($(CONFIG_ARCH_FPU),y)
CSRCS += lpc43_ostest.c
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

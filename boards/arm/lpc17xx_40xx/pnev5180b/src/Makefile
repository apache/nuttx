##############################################################################
# boards/arm/lpc17xx_40xx/pnev5180b/src/Makefile
#
#   Copyright (C) 2019 Gregory Nutt. All rights reserved.
#   Author: Michael Jung <mijung@gmx.net>
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
##############################################################################

include $(TOPDIR)/Make.defs

CSRCS = lpc17_40_boot.c lpc17_40_bringup.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += lpc17_40_leds.c
endif

ifeq ($(CONFIG_LIB_BOARDCTL),y)
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

############################################################################
# boards/arm/lpc31xx/ea3131/src/Makefile
#
#   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
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

CSRCS = lpc31_boot.c lpc31_clkinit.c

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += lpc31_buttons.c
endif

ifeq ($(CONFIG_LPC31_EXTDRAM),y)
CSRCS += lpc31_mem.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += lpc31_leds.c
endif

ifeq ($(CONFIG_LPC31_SPI),y)
CSRCS += lpc31_spi.c
endif

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += lpc31_appinit.c
endif

ifeq ($(CONFIG_PAGING),y)
CSRCS += lpc31_fillpage.c
endif

ifeq ($(CONFIG_LPC31_USBOTG),y)
ifeq ($(CONFIG_USBHOST),y)
CSRCS += lpc31_usbhost.c
endif
endif

include $(TOPDIR)/boards/Board.mk

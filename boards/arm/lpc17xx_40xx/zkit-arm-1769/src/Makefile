############################################################################
# boards/arm/lpc17xx_40xx/zkit-arm-1769/src/Makefile
#
#   Copyright (C) 2013 Zilogic Systems. All rights reserved.
#   Author: BabuSubashChandar <code@zilogic.com>
#
# Based on boards/lpcxpresso-lpc1768/src/Makefile
#
#   Copyright (C) 201, 2016 Gregory Nutt. All rights reserved.
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

CSRCS = lpc17_40_boot.c lpc17_40_leds.c lpc17_40_ssp.c lpc17_40_buttons.c lpc17_40_lcd.c

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += lpc17_40_appinit.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += lpc17_40_usbmsc.c
endif

ifeq ($(CONFIG_ADC),y)
CSRCS += lpc17_40_adc.c
endif

ifeq ($(CONFIG_DAC),y)
CSRCS += lpc17_40_dac.c
endif

ifeq ($(CONFIG_CAN),y)
CSRCS += lpc17_40_can.c
endif

ifeq ($(CONFIG_LPC17_40_SPI),y)
CSRCS += lpc17_40_spi.c
endif

include $(TOPDIR)/boards/Board.mk

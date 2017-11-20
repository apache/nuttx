############################################################################
# configs/open1788/src/Makefile
#
#   Copyright (C) 2013, 2017 Gregory Nutt. All rights reserved.
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

-include $(TOPDIR)/Make.defs

ASRCS =
CSRCS = lpc17_boardinitialize.c lpc17_bringup.c lpc17_ssp.c

ifeq ($(CONFIG_LPC17_EMC),y)
ifeq ($(CONFIG_LPC17_EXTNOR),y)
  CSRCS += lpc17_norinitialize.c
endif

ifeq ($(CONFIG_LPC17_EXTNAND),y)
  CSRCS += lpc17_nandinitialize.c
endif

ifeq ($(CONFIG_LPC17_EXTDRAM),y)
  CSRCS += lpc17_sdraminitialize.c
endif
endif

ifeq ($(CONFIG_LPC17_LCD),y)
  CSRCS += lpc17_lcd.c
endif

ifeq ($(CONFIG_LIB_BOARDCTL),y)
  CSRCS += lpc17_appinit.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
  CSRCS += lpc17_autoleds.c
else
  CSRCS += lpc17_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
  CSRCS += lpc17_buttons.c
else ifeq ($(CONFIG_OPEN1788_DJOYSTICK),y)
  CSRCS += lpc17_djoystick.c
endif

ifeq ($(CONFIG_INPUT_ADS7843E),y)
  CSRCS += lpc17_touchscreen.c
endif

include $(TOPDIR)/configs/Board.mk

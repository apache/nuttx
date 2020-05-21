############################################################################
# boards/arm/sam34/sam4l-xplained/src/Makefile
#
#   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

CSRCS = sam_boot.c

ifeq ($(CONFIG_SAM34_SPI0),y)
CSRCS += sam_spi.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += sam_autoleds.c
else
CSRCS += sam_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += sam_buttons.c
endif

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += sam_appinit.c
endif

ifeq ($(CONFIG_SAM34_LCDCA),y)
ifeq ($(CONFIG_SAM4L_XPLAINED_SLCD1MODULE),y)
CSRCS += sam_slcd.c
endif
endif

ifeq ($(CONFIG_SAM34_SPI0),y)
ifeq ($(CONFIG_SAM4L_XPLAINED_IOMODULE),y)
CSRCS += sam_mmcsd.c
endif
endif

ifeq ($(CONFIG_SAM34_SPI0),y)
ifeq ($(CONFIG_SAM4L_XPLAINED_OLED1MODULE),y)
ifeq ($(CONFIG_LCD_UG2832HSWEG04),y)
CSRCS += sam_ug2832hsweg04.c
endif
endif
endif

include $(TOPDIR)/boards/Board.mk

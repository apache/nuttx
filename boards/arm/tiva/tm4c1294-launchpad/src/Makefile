############################################################################
# boards/arm/tiva/tm4c1294-launchpad/src/Makefile
#
#   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

CSRCS = tm4c_boot.c tm4c_bringup.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += tm4c_autoleds.c
else
CSRCS += tm4c_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += tm4c_buttons.c
endif

ifeq ($(CONFIG_TIVA_ETHERNET),y)
CSRCS += tm4c_ethernet.c
endif

ifeq ($(CONFIG_DK_TM4C129X_TIMER),y)
CSRCS += tm4c_timer.c
endif

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += tm4c_appinit.c
endif

ifeq ($(CONFIG_TIVA_HCIUART),y)
ifeq ($(CONFIG_BLUETOOTH_UART),y)
CSRCS += tm4c_hciuart.c
endif
endif

include $(TOPDIR)/boards/Board.mk

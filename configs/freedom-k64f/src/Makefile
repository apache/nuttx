############################################################################
# configs/freedom-k64f/src/Makefile
#
#   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
CSRCS = k64_boot.c k64_spi.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += k64_autoleds.c
else
CSRCS += k64_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += k64_buttons.c
endif

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += k64_appinit.c k64_bringup.c
else ifeq ($(CONFIG_BOARD_INITIALIZE),y)
CSRCS += k64_bringup.c
endif

ifeq ($(CONFIG_KINETIS_SDHC),y)
CSRCS += k64_sdhc.c
ifeq ($(CONFIG_FS_AUTOMOUNTER),y)
CSRCS += k64_automount.c
endif
endif

ifeq ($(CONFIG_USBDEV),y)
CSRCS += k64_usbdev.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += k64_usbmsc.c
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += k64_pwm.c
endif

ifeq ($(CONFIG_BOARDCTL_UNIQUEID),y)
CSRCS += k64_uid.c
endif

include $(TOPDIR)/configs/Board.mk

############################################################################
# boards/arm/samd5e5/metro-m4/src/Makefile
#
#   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

CSRCS = sam_boot.c sam_bringup.c

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += sam_appinit.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += sam_autoleds.c
else
CSRCS += sam_userleds.c
endif

ifeq ($(CONFIG_USBDEV),y)
CSRCS += sam_usbdev.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += sam_usbmsc.c
endif

ifeq ($(CONFIG_USBDEV_COMPOSITE),y)
CSRCS += sam_composite.c
endif

ifeq ($(CONFIG_USBHOST),y)
CSRCS += sam_usbhost.c
endif

ifeq ($(CONFIG_FS_AUTOMOUNTER),y)
CSRCS += sam_automount.c
endif

ifeq ($(CONFIG_SAMD5E5_SERCOM5_ISI2C),y)
CSRCS += sam_i2c.c
ifeq ($(CONFIG_MTD_AT24XX),y)
CSRCS += sam_at24.c
endif
endif

ifeq ($(CONFIG_FS_SMARTFS),y)
CSRCS += sam_smartfs.c
endif

ifeq ($(CONFIG_BQ27426),y)
CSRCS += sam_bq27426.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
CSRCS += sam_gpio.c
endif

include $(TOPDIR)/boards/Board.mk

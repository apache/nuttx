############################################################################
# boards/xtensa/esp32/esp32-core/src/Makefile
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

include $(TOPDIR)/Make.defs

SCRIPTDIR = $(BOARD_DIR)$(DELIM)scripts

CONFIGFILE = $(TOPDIR)$(DELIM)include$(DELIM)nuttx$(DELIM)config.h

CSRCS = esp32_boot.c esp32_bringup.c

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += esp32_appinit.c
ifeq ($(CONFIG_BOARDCTL_RESET),y)
CSRCS += esp32_reset.c
endif
endif

ifeq ($(CONFIG_ESP32_SPI),y)
CSRCS += esp32_spi.c
endif

ifeq ($(CONFIG_MMCSD),y)
CSRCS += esp32_mmcsd.c
endif

ifeq ($(CONFIG_ESP32_SPIFLASH),y)
CSRCS += esp32_spiflash.c
endif

ifeq ($(CONFIG_TIMER),y)
ifeq ($(CONFIG_ESP32_TIMER),y)
CSRCS += esp32_timer.c
endif
endif

SCRIPTIN = $(SCRIPTDIR)$(DELIM)esp32.template.ld
SCRIPTOUT = $(SCRIPTDIR)$(DELIM)esp32_out.ld

EXTRA_DISTCLEAN = $(call DELFILE, $(SCRIPTOUT))

.PHONY = context

include $(TOPDIR)/boards/Board.mk

$(SCRIPTOUT): $(SCRIPTIN) $(CONFIGFILE)
	$(Q) $(CC) -isystem $(TOPDIR)/include -C -P -x c -E $(SCRIPTIN) -o $@

context:: $(SCRIPTOUT)

############################################################################
# configs//srp32-core/Makefile
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

SCRIPTDIR = $(TOPDIR)$(DELIM)configs$(DELIM)$(CONFIG_ARCH_BOARD)$(DELIM)scripts
CONFIGFILE = $(TOPDIR)$(DELIM)include$(DELIM)nuttx$(DELIM)config.h

ASRCS =
CSRCS = esp32_boot.c esp32_bringup.c

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += esp32_appinit.c
endif

SCRIPTIN = $(SCRIPTDIR)$(DELIM)esp32.template
SCRIPTOUT = $(SCRIPTDIR)$(DELIM)esp32_out.ld

BOARD_CONTEXT = y
EXTRA_CLEAN =
EXTRA_DISTCLEAN = $(call DELFILE, $(SCRIPTOUT))

.PHONY = context

include $(TOPDIR)/configs/Board.mk

$(SCRIPTOUT): $(SCRIPTIN) $(CONFIGFILE)
	# $(call PREPROCESS, $(SCRIPTIN), $@)
	$(Q) $(CC) -isystem $(TOPDIR)/include -C -P -x c -E $(SCRIPTIN) -o $@

context: $(SCRIPTOUT)

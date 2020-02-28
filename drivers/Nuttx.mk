############################################################################
# drivers/Nuttx.mk
#
#   Copyright (C) 2007-2014, 2016 Gregory Nutt. All rights reserved.
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

-include $(TOPDIR)/Nuttx.defs
DELIM ?= $(strip /)

ifeq ($(WINTOOL),y)
INCDIROPT = -w
endif

DEPPATH = --dep-path .
ASRCS =
CSRCS =
VPATH = .

# Include support for various drivers.  Each Nuttx.defs file will add its
# files to the source file list, add its DEPPATH info, and will add
# the appropriate paths to the VPATH variable

include analog$(DELIM)Nuttx.defs
include audio$(DELIM)Nuttx.defs
include bch$(DELIM)Nuttx.defs
include can$(DELIM)Nuttx.defs
include crypto$(DELIM)Nuttx.defs
include i2c$(DELIM)Nuttx.defs
include i2s$(DELIM)Nuttx.defs
include input$(DELIM)Nuttx.defs
include ioexpander$(DELIM)Nuttx.defs
include lcd$(DELIM)Nuttx.defs
include leds$(DELIM)Nuttx.defs
include loop$(DELIM)Nuttx.defs
include mmcsd$(DELIM)Nuttx.defs
include modem$(DELIM)Nuttx.defs
include mtd$(DELIM)Nuttx.defs
include eeprom$(DELIM)Nuttx.defs
include net$(DELIM)Nuttx.defs
include pipes$(DELIM)Nuttx.defs
include power$(DELIM)Nuttx.defs
include rptun$(DELIM)Nuttx.defs
include sensors$(DELIM)Nuttx.defs
include serial$(DELIM)Nuttx.defs
include spi$(DELIM)Nuttx.defs
include syslog$(DELIM)Nuttx.defs
include timers$(DELIM)Nuttx.defs
include usbdev$(DELIM)Nuttx.defs
include usbhost$(DELIM)Nuttx.defs
include usbmisc$(DELIM)Nuttx.defs
include usbmonitor$(DELIM)Nuttx.defs
include video$(DELIM)Nuttx.defs
include wireless$(DELIM)Nuttx.defs
include contactless$(DELIM)Nuttx.defs
include 1wire$(DELIM)Nuttx.defs
include rf$(DELIM)Nuttx.defs

ifeq ($(CONFIG_SPECIFIC_DRIVERS),y)
include platform$(DELIM)Nuttx.defs
endif

ifeq ($(CONFIG_DEV_SIMPLE_ADDRENV),y)
  CSRCS += addrenv.c
endif

CSRCS += dev_null.c dev_zero.c

ifneq ($(CONFIG_DISABLE_MOUNTPOINT),y)
  CSRCS += ramdisk.c
ifeq ($(CONFIG_DRVR_MKRD),y)
  CSRCS += mkrd.c
endif
ifeq ($(CONFIG_DRVR_WRITEBUFFER),y)
  CSRCS += rwbuffer.c
else
ifeq ($(CONFIG_DRVR_READAHEAD),y)
  CSRCS += rwbuffer.c
endif
endif
endif

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = libdrivers$(LIBEXT)

all: $(BIN)
.PHONY: depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Nuttx.mk $(SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Nuttx.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Nuttx.dep)
	$(call DELFILE, .depend)

-include Nuttx.dep

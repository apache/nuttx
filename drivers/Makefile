############################################################################
# drivers/Makefile
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

include $(TOPDIR)/Make.defs

# Include support for various drivers.  Each Make.defs file will add its
# files to the source file list, add its DEPPATH info, and will add
# the appropriate paths to the VPATH variable

include analog/Make.defs
include audio/Make.defs
include bch/Make.defs
include can/Make.defs
include crypto/Make.defs
include i2c/Make.defs
include i2s/Make.defs
include input/Make.defs
include ioexpander/Make.defs
include lcd/Make.defs
include leds/Make.defs
include loop/Make.defs
include mmcsd/Make.defs
include modem/Make.defs
include mtd/Make.defs
include eeprom/Make.defs
include net/Make.defs
include note/Make.defs
include pipes/Make.defs
include power/Make.defs
include rptun/Make.defs
include sensors/Make.defs
include serial/Make.defs
include spi/Make.defs
include syslog/Make.defs
include timers/Make.defs
include usbdev/Make.defs
include usbhost/Make.defs
include usbmisc/Make.defs
include usbmonitor/Make.defs
include video/Make.defs
include wireless/Make.defs
include contactless/Make.defs
include 1wire/Make.defs
include rf/Make.defs

ifeq ($(CONFIG_SPECIFIC_DRIVERS),y)
include platform/Make.defs
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

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

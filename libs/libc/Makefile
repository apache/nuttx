############################################################################
# libs/libc/Makefile
#
#   Copyright (C) 2007-2014, 2016-2017 Gregory Nutt. All rights reserved.
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
###########################################################################

-include $(TOPDIR)/Make.defs

# CFLAGS

ifneq ($(CONFIG_BUILD_FLAT),y)
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  KDEFINE = ${shell $(TOPDIR)\tools\define.bat "$(CC)" __KERNEL__}
else
  KDEFINE = ${shell $(TOPDIR)/tools/define.sh "$(CC)" __KERNEL__}
endif
endif

# Sources and paths

ASRCS =
CSRCS =

DEPPATH := --dep-path .
VPATH := .

include aio/Make.defs
include audio/Make.defs
include dirent/Make.defs
include dllfcn/Make.defs
include endian/Make.defs
include fixedmath/Make.defs
include hex2bin/Make.defs
include inttypes/Make.defs
include libgen/Make.defs
include locale/Make.defs
include lzf/Make.defs
include machine/Make.defs
include math/Make.defs
include misc/Make.defs
include modlib/Make.defs
include net/Make.defs
include netdb/Make.defs
include pthread/Make.defs
include queue/Make.defs
include sched/Make.defs
include semaphore/Make.defs
include signal/Make.defs
include spawn/Make.defs
include stdio/Make.defs
include stdlib/Make.defs
include string/Make.defs
include symtab/Make.defs
include syslog/Make.defs
include termios/Make.defs
include time/Make.defs
include tls/Make.defs
include uio/Make.defs
include unistd/Make.defs
include userfs/Make.defs
include wchar/Make.defs
include wctype/Make.defs
include wqueue/Make.defs

# REVISIT: Backslash causes problems in $(COBJS) target
DELIM := $(strip /)
BINDIR ?= bin

AOBJS = $(patsubst %.S, $(BINDIR)$(DELIM)%$(OBJEXT), $(ASRCS))
COBJS = $(patsubst %.c, $(BINDIR)$(DELIM)%$(OBJEXT), $(CSRCS))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

UBIN = libuc$(LIBEXT)
KBIN = libkc$(LIBEXT)
BIN ?= libc$(LIBEXT)

all: $(BIN)
.PHONY: clean distclean

$(AOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)
	$(call MOVEOBJ,$(patsubst $(BINDIR)$(DELIM)%$(OBJEXT),%,$@),$(BINDIR))

$(COBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.c
	$(call COMPILE, $<, $@)
	$(call MOVEOBJ,$(patsubst $(BINDIR)$(DELIM)%$(OBJEXT),%,$@),$(BINDIR))

# C library for the flat build

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))
ifeq ($(CONFIG_LIB_ZONEINFO_ROMFS),y)
	$(Q) $(MAKE) -C zoneinfo all TOPDIR=$(TOPDIR) BIN=$(BIN)
endif

# C library for the user phase of the two-pass kernel build

ifneq ($(BIN),$(UBIN))
$(UBIN):
	$(Q) $(MAKE) $(UBIN) BIN=$(UBIN) BINDIR=ubin TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
endif

# C library for the kernel phase of the two-pass kernel build

ifneq ($(BIN),$(KBIN))
$(KBIN):
	$(Q) $(MAKE) $(KBIN) BIN=$(KBIN) BINDIR=kbin TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
endif

# Context

context:
ifeq ($(CONFIG_LIB_ZONEINFO_ROMFS),y)
	$(Q) $(MAKE) -C zoneinfo context TOPDIR=$(TOPDIR) BIN=$(BIN)
endif

# Dependencies

.depend: Makefile $(SRCS)
ifeq ($(CONFIG_BUILD_FLAT),y)
	$(Q) $(MKDEP) --obj-path bin --obj-suffix $(OBJEXT) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make_bin.dep
else
	$(Q) $(MKDEP) --obj-path ubin --obj-suffix $(OBJEXT) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make_ubin.dep
	$(Q) $(MKDEP) --obj-path kbin --obj-suffix $(OBJEXT) $(DEPPATH) "$(CC)" -- $(CFLAGS) $(KDEFINE) -- $(SRCS) >Make_kbin.dep
endif
ifeq ($(CONFIG_LIB_ZONEINFO_ROMFS),y)
	$(Q) $(MAKE) -C zoneinfo depend TOPDIR=$(TOPDIR) BIN=$(BIN)
endif
	$(Q) touch $@

depend: .depend

# Clean most derived files, retaining the configuration

clean:
	$(Q) $(MAKE) -C bin  clean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C ubin clean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C kbin clean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C zoneinfo clean TOPDIR=$(TOPDIR) BIN=$(BIN)
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(UBIN))
	$(call DELFILE, $(KBIN))
	$(call CLEAN)

# Deep clean -- removes all traces of the configuration

distclean: clean
	$(Q) $(MAKE) -C bin  distclean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C ubin distclean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C kbin distclean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C zoneinfo distclean TOPDIR=$(TOPDIR) BIN=$(BIN)
	$(call DELFILE, Make_bin.dep)
	$(call DELFILE, Make_ubin.dep)
	$(call DELFILE, Make_kbin.dep)
	$(call DELFILE, .depend)

-include Make_bin.dep
-include Make_ubin.dep
-include Make_kbin.dep

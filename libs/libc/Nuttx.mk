############################################################################
# libs/libc/Nuttx.mk
#
#   Copyright (C) 2007-2014, 2016-2017, 2019 Gregory Nutt. All rights reserved.
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

-include $(TOPDIR)/Nuttx.defs

# CFLAGS

ifneq ($(CONFIG_BUILD_FLAT),y)
  KDEFINE = ${shell $(DEFINE) "$(CC)" __KERNEL__}
endif

# Sources and paths

ASRCS =
CSRCS =

DEPPATH := --dep-path .
VPATH := .

include aio/Nuttx.defs
include audio/Nuttx.defs
include builtin/Nuttx.defs
include dirent/Nuttx.defs
include dlfcn/Nuttx.defs
include endian/Nuttx.defs
include fixedmath/Nuttx.defs
include grp/Nuttx.defs
include hex2bin/Nuttx.defs
include inttypes/Nuttx.defs
include libgen/Nuttx.defs
include locale/Nuttx.defs
include lzf/Nuttx.defs
include machine/Nuttx.defs
include math/Nuttx.defs
include misc/Nuttx.defs
include modlib/Nuttx.defs
include net/Nuttx.defs
include netdb/Nuttx.defs
include pthread/Nuttx.defs
include pwd/Nuttx.defs
include queue/Nuttx.defs
include sched/Nuttx.defs
include semaphore/Nuttx.defs
include signal/Nuttx.defs
include spawn/Nuttx.defs
include stdio/Nuttx.defs
include stdlib/Nuttx.defs
include string/Nuttx.defs
include symtab/Nuttx.defs
include syslog/Nuttx.defs
include termios/Nuttx.defs
include time/Nuttx.defs
include tls/Nuttx.defs
include uio/Nuttx.defs
include unistd/Nuttx.defs
include userfs/Nuttx.defs
include wchar/Nuttx.defs
include wctype/Nuttx.defs
include wqueue/Nuttx.defs

# Rule for the symbol table generation

MKSYMTAB = $(TOPDIR)$(DELIM)tools$(DELIM)mksymtab$(HOSTEXEEXT)

$(MKSYMTAB):
	$(Q) $(MAKE) -C $(TOPDIR)$(DELIM)tools -f Nuttx.mk.host mksymtab

# C library and math library symbols should be available in the FLAT
# and PROTECTED builds.  KERNEL builds are separately linked and so should
# not need symbol tables.

CSVFILES  = $(TOPDIR)$(DELIM)libs$(DELIM)libc$(DELIM)libc.csv
CSVFILES += $(TOPDIR)$(DELIM)libs$(DELIM)libc$(DELIM)math.csv

# In the PROTECTED and KERNEL builds, the applications could link with
# libproxy which will provide symbol-compatible access to OS functions
# via a call gate, but the applications which link with these functions
# directly could remove the repeat proxy code to save the space.

CSVFILES += $(TOPDIR)$(DELIM)syscall$(DELIM)syscall.csv

ifeq ($(CONFIG_EXECFUNCS_SYSTEM_SYMTAB),y)

exec_symtab.c : $(CSVFILES) $(MKSYMTAB)
	$(Q) cat $(CSVFILES) | LC_ALL=C sort >$@.csv
	$(Q) $(MKSYMTAB) $@.csv $@ $(CONFIG_EXECFUNCS_SYMTAB_ARRAY) $(CONFIG_EXECFUNCS_NSYMBOLS_VAR)
	$(Q) rm -f $@.csv

CSRCS += exec_symtab.c

endif

ifeq ($(CONFIG_MODLIB_SYSTEM_SYMTAB),y)

modlib_symtab.c : $(CSVFILES) $(MKSYMTAB)
	$(Q) cat $(CSVFILES) | LC_ALL=C sort >$@.csv
	$(Q) $(MKSYMTAB) $@.csv $@ $(CONFIG_MODLIB_SYMTAB_ARRAY) $(CONFIG_MODLIB_NSYMBOLS_VAR)
	$(Q) rm -f $@.csv

CSRCS += modlib_symtab.c

endif

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

$(COBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

# C library for the flat build

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))
ifeq ($(CONFIG_LIB_ZONEINFO_ROMFS),y)
	$(Q) $(MAKE) -C zoneinfo -f Nuttx.mk all TOPDIR=$(TOPDIR) BIN=$(BIN)
endif

# C library for the user phase of the two-pass kernel build

ifneq ($(BIN),$(UBIN))
$(UBIN):
	$(Q) $(MAKE) -f Nuttx.mk $(UBIN) BIN=$(UBIN) BINDIR=ubin TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
endif

# C library for the kernel phase of the two-pass kernel build

ifneq ($(BIN),$(KBIN))
$(KBIN):
	$(Q) $(MAKE) -f Nuttx.mk $(KBIN) BIN=$(KBIN) BINDIR=kbin TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
endif

# Context

context:
ifeq ($(CONFIG_LIB_ZONEINFO_ROMFS),y)
	$(Q) $(MAKE) -C zoneinfo -f Nuttx.mk context TOPDIR=$(TOPDIR) BIN=$(BIN)
endif

# Dependencies

.depend: Nuttx.mk $(SRCS)
ifeq ($(CONFIG_BUILD_FLAT),y)
	$(Q) $(MKDEP) --obj-path bin --obj-suffix $(OBJEXT) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >bin/Nuttx.dep
else
	$(Q) $(MKDEP) --obj-path ubin --obj-suffix $(OBJEXT) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >ubin/Nuttx.dep
	$(Q) $(MKDEP) --obj-path kbin --obj-suffix $(OBJEXT) $(DEPPATH) "$(CC)" -- $(CFLAGS) $(KDEFINE) -- $(SRCS) >kbin/Nuttx.dep
endif
ifeq ($(CONFIG_LIB_ZONEINFO_ROMFS),y)
	$(Q) $(MAKE) -C zoneinfo -f Nuttx.mk depend TOPDIR=$(TOPDIR) BIN=$(BIN)
endif
	$(Q) touch $@

depend: .depend

# Clean most derived files, retaining the configuration

clean:
	$(Q) $(MAKE) -C bin  -f Nuttx.mk clean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C ubin -f Nuttx.mk clean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C kbin -f Nuttx.mk clean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C zoneinfo -f Nuttx.mk clean TOPDIR=$(TOPDIR) BIN=$(BIN)
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(UBIN))
	$(call DELFILE, $(KBIN))
	$(call CLEAN)

# Deep clean -- removes all traces of the configuration

distclean: clean
	$(Q) $(MAKE) -C bin  -f Nuttx.mk distclean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C ubin -f Nuttx.mk distclean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C kbin -f Nuttx.mk distclean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C zoneinfo -f Nuttx.mk distclean TOPDIR=$(TOPDIR) BIN=$(BIN)
	$(call DELFILE, bin/Nuttx.dep)
	$(call DELFILE, ubin/Nuttx.dep)
	$(call DELFILE, kbin/Nuttx.dep)
	$(call DELFILE, .depend)

-include bin/Nuttx.dep
-include ubin/Nuttx.dep
-include kbin/Nuttx.dep

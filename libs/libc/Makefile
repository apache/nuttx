############################################################################
# libs/libc/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
###########################################################################

include $(TOPDIR)/Make.defs

include aio/Make.defs
include assert/Make.defs
include audio/Make.defs
include builtin/Make.defs
include ctype/Make.defs
include dirent/Make.defs
include dlfcn/Make.defs
include errno/Make.defs
include eventfd/Make.defs
include fixedmath/Make.defs
include grp/Make.defs
include hex2bin/Make.defs
include inttypes/Make.defs
include libgen/Make.defs
include locale/Make.defs
include lzf/Make.defs
include machine/Make.defs
include misc/Make.defs
include modlib/Make.defs
include net/Make.defs
include netdb/Make.defs
include obstack/Make.defs
include pthread/Make.defs
include pwd/Make.defs
include queue/Make.defs
include regex/Make.defs
include sched/Make.defs
include semaphore/Make.defs
include signal/Make.defs
include spawn/Make.defs
include stdio/Make.defs
include stdlib/Make.defs
include stream/Make.defs
include string/Make.defs
include symtab/Make.defs
include syslog/Make.defs
include termios/Make.defs
include time/Make.defs
include tls/Make.defs
include uio/Make.defs
include unistd/Make.defs
include userfs/Make.defs
include uuid/Make.defs
include wchar/Make.defs
include wctype/Make.defs
include wqueue/Make.defs

# Use double delim to fix windows native build and give an error:
# makefile:132: *** target mode do not include“%”. stop.
#
# In Windows environment DELIM := $(strip \) but \ has two role:
# first: \ as directory, and second \ as Escape character, Reference:
#
# https://github.com/apache/nuttx/pull/7572#discussion_r1028219229

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  DELIMS = $(DELIM)$(DELIM)
else
  DELIMS = $(DELIM)
endif

CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)libs$(DELIM)libc

# Rule for the symbol table generation

MKSYMTAB = $(TOPDIR)$(DELIM)tools$(DELIM)mksymtab$(HOSTEXEEXT)

$(MKSYMTAB):
	$(Q) $(MAKE) -C $(TOPDIR)$(DELIM)tools -f Makefile.host mksymtab

# C library and math library symbols should be available in the FLAT
# and PROTECTED builds.  KERNEL builds are separately linked and so should
# not need symbol tables.

CSVFILES  = $(TOPDIR)$(DELIM)libs$(DELIM)libc$(DELIM)libc.csv
CSVFILES += $(TOPDIR)$(DELIM)libs$(DELIM)libm$(DELIM)libm.csv

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

modlib_sys_symtab.c : $(CSVFILES) $(MKSYMTAB)
	$(Q) cat $(CSVFILES) | LC_ALL=C sort >$@.csv
	$(Q) $(MKSYMTAB) $@.csv $@ $(CONFIG_MODLIB_SYMTAB_ARRAY) $(CONFIG_MODLIB_NSYMBOLS_VAR)
	$(Q) rm -f $@.csv

CSRCS += modlib_sys_symtab.c

endif

BINDIR ?= bin

AOBJS = $(patsubst %.S, $(BINDIR)$(DELIMS)%$(OBJEXT), $(ASRCS))
COBJS = $(patsubst %.c, $(BINDIR)$(DELIMS)%$(OBJEXT), $(CSRCS))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

KBIN = libkc$(LIBEXT)
BIN ?= libc$(LIBEXT)

all: $(BIN)
.PHONY: clean distclean

$(AOBJS): $(BINDIR)$(DELIMS)%$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

# REVISIT: Backslash causes problems in $(COBJS) target

$(COBJS): $(BINDIR)$(DELIMS)%$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

# C library for the flat build and
# the user phase of the two-pass kernel build

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))
ifeq ($(CONFIG_LIBC_ZONEINFO_ROMFS),y)
	$(Q) $(MAKE) -C zoneinfo all BIN=$(BIN)
endif

# C library for the kernel phase of the two-pass kernel build

ifneq ($(BIN),$(KBIN))
$(KBIN): $(OBJS)
	$(Q) $(MAKE) $(KBIN) BIN=$(KBIN) EXTRAFLAGS="$(EXTRAFLAGS)"
endif

# Context

context::
ifeq ($(CONFIG_LIBC_ZONEINFO_ROMFS),y)
	$(Q) $(MAKE) -C zoneinfo context BIN=$(BIN)
endif

# Dependencies

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, bin/Make.dep, $^)
	$(call DELFILE, $^)

makekdepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, kbin/Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MAKE) makedepfile OBJPATH="bin"
ifneq ($(CONFIG_BUILD_FLAT),y)
	$(Q) $(MAKE) makekdepfile CFLAGS="$(CFLAGS) $(KDEFINE)" OBJPATH="kbin"
endif
ifeq ($(CONFIG_LIBC_ZONEINFO_ROMFS),y)
	$(Q) $(MAKE) -C zoneinfo depend BIN=$(BIN)
endif
	$(Q) touch $@

depend:: .depend

# Clean most derived files, retaining the configuration

clean::
	$(Q) $(MAKE) -C bin  clean
	$(Q) $(MAKE) -C kbin clean
	$(Q) $(MAKE) -C zoneinfo clean BIN=$(BIN)
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(KBIN))
	$(call CLEAN)

# Deep clean -- removes all traces of the configuration

distclean:: clean
	$(Q) $(MAKE) -C bin  distclean
	$(Q) $(MAKE) -C kbin distclean
	$(Q) $(MAKE) -C zoneinfo distclean BIN=$(BIN)
	$(call DELFILE, exec_symtab.c)
	$(call DELFILE, bin$(DELIM)Make.dep)
	$(call DELFILE, kbin$(DELIM)Make.dep)
	$(call DELFILE, .depend)

-include bin$(DELIM)Make.dep
-include kbin$(DELIM)Make.dep

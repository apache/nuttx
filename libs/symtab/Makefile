############################################################################
# libs/symtab/Makefile
#
#   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

MKSYMTAB = $(TOPDIR)$(DELIM)tools$(DELIM)mksymtab$(HOSTEXEEXT)

# C library and math library symbols should be available in the FLAT
# and PROTECTED builds.  KERNEL builds are separately linked and so should
# not need symbol tables.

CSVFILES  = $(TOPDIR)$(DELIM)libs$(DELIM)libc$(DELIM)libc.csv
CSVFILES += $(TOPDIR)$(DELIM)libs$(DELIM)libc$(DELIM)math.csv

# In the PROTECTED and KERNEL builds, the syscall symbol table is not
# necessary.  In those build modes, the applications should link with
# libproxy which will provide symbol-compatible access to OS functions
# via a call gate.

ifeq ($(CONFIG_BUILD_FLAT),y)
CSVFILES += $(TOPDIR)$(DELIM)syscall$(DELIM)syscall.csv
endif

DEPPATH = --dep-path .

SYMTAB_CSRCS  =

BIN = libsymtab$(LIBEXT)

ifeq ($(CONFIG_EXECFUNCS_SYSTEM_SYMTAB),y)

all: symtab.c $(BIN)
.PHONY: all

SYMTAB_CSRCS += symtab.c
SYMTAB_COBJS = $(SYMTAB_CSRCS:.c=$(OBJEXT))

SYMTAB_SRCS = $(SYMTAB_CSRCS)
SYMTAB_OBJS = $(SYMTAB_COBJS)

%$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(SYMTAB_OBJS)
	$(call ARCHIVE, $@, $(SYMTAB_OBJS))

$(MKSYMTAB):
	$(Q) $(MAKE) -C $(TOPDIR)$(DELIM)tools -f Makefile.host mksymtab

symtab.c : $(CSVFILES) $(MKSYMTAB)
	$(Q) rm -f tmp.csv $@
	$(Q) cat $(CSVFILES) | LC_ALL=C sort >tmp.csv
	$(Q) $(MKSYMTAB) tmp.csv $@
	$(Q) echo >>$@
	$(Q) echo >>$@ 'int g_nsymbols = NSYMBOLS;'

else
all:
endif

.depend: Makefile $(SYMTAB_SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SYMTAB_SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))

distclean: clean
	$(call DELFILE, symtab.c)
	$(call DELFILE, tmp.csv)

-include Make.dep

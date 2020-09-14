############################################################################
# mm/Makefile
#
#   Copyright (C) 2007, 2012, 2013-2014, 2017 Gregory Nutt. All rights reserved.
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

# Sources and paths

include mm_heap/Make.defs
include umm_heap/Make.defs
include kmm_heap/Make.defs
include mm_gran/Make.defs
include shm/Make.defs
include iob/Make.defs

BINDIR ?= bin

CFLAGS += ${shell $(INCDIR) "$(CC)" $(TOPDIR)$(DELIM)mm}

AOBJS = $(patsubst %.S, $(BINDIR)$(DELIM)%$(OBJEXT), $(ASRCS))
COBJS = $(patsubst %.c, $(BINDIR)$(DELIM)%$(OBJEXT), $(CSRCS))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

KBIN = libkmm$(LIBEXT)
BIN ?= libmm$(LIBEXT)

all: $(BIN)
.PHONY: clean distclean

$(AOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

# Memory manager for the flat build and
# the user phase of the two-pass kernel build

$(BIN):	$(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

# Memory manager for the kernel phase of the two-pass kernel build

ifneq ($(BIN),$(KBIN))
$(KBIN):
	$(Q) $(MAKE) $(KBIN) BIN=$(KBIN) BINDIR=kbin EXTRAFLAGS="$(EXTRAFLAGS)"
endif

# Dependencies

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MKDEP) --obj-path bin --obj-suffix $(OBJEXT) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >bin/Make.dep
ifneq ($(CONFIG_BUILD_FLAT),y)
	$(Q) $(MKDEP) --obj-path kbin --obj-suffix $(OBJEXT) $(DEPPATH) "$(CC)" -- $(CFLAGS) $(KDEFINE) -- $(SRCS) >kbin/Make.dep
endif
	$(Q) touch $@

depend: .depend

# Clean most derived files, retaining the configuration

clean:
	$(Q) $(MAKE) -C bin  clean
	$(Q) $(MAKE) -C kbin clean
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(KBIN))
	$(call CLEAN)

# Deep clean -- removes all traces of the configuration

distclean: clean
	$(Q) $(MAKE) -C bin  distclean
	$(Q) $(MAKE) -C kbin distclean
	$(call DELFILE, bin/Make.dep)
	$(call DELFILE, kbin/Make.dep)
	$(call DELFILE, .depend)

-include bin/Make.dep
-include kbin/Make.dep

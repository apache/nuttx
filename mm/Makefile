############################################################################
# mm/Makefile
#
#   Copyright (C) 2007, 2012, 2013-2014 Gregory Nutt. All rights reserved.
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

# REVISIT: Backslash causes problems in $(COBJS) target
DELIM := $(strip /)

# CFLAGS

ifeq ($(CONFIG_NUTTX_KERNEL),y)
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  KDEFINE = ${shell $(TOPDIR)\tools\define.bat "$(CC)" __KERNEL__}
else
  KDEFINE = ${shell $(TOPDIR)/tools/define.sh "$(CC)" __KERNEL__}
endif
endif

# Core allocator logic

ASRCS  =
CSRCS  = mm_initialize.c mm_sem.c  mm_addfreechunk.c mm_size2ndx.c
CSRCS += mm_shrinkchunk.c mm_malloc.c mm_zalloc.c mm_calloc.c mm_realloc.c
CSRCS += mm_memalign.c mm_free.c mm_mallinfo.c

# Allocator instances

CSRCS += mm_user.c
ifeq ($(CONFIG_NUTTX_KERNEL),y)
ifeq ($(CONFIG_MM_KERNEL_HEAP),y)
CSRCS += mm_kernel.c
endif
endif

# An optional granule allocator

ifeq ($(CONFIG_GRAN),y)
CSRCS += mm_graninit.c mm_granalloc.c mm_granfree.c mm_grancritical.c
endif

BINDIR ?= bin

AOBJS = $(patsubst %.S, $(BINDIR)$(DELIM)%$(OBJEXT), $(ASRCS))
COBJS = $(patsubst %.c, $(BINDIR)$(DELIM)%$(OBJEXT), $(CSRCS))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

UBIN = libumm$(LIBEXT)
KBIN = libkmm$(LIBEXT)
BIN ?= libmm$(LIBEXT)

all: $(BIN)
.PHONY: clean distclean

$(AOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)
	$(call MOVEOBJ,$(patsubst $(BINDIR)$(DELIM)%$(OBJEXT),%,$@),$(BINDIR))

$(COBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.c
	$(call COMPILE, $<, $@)
	$(call MOVEOBJ,$(patsubst $(BINDIR)$(DELIM)%$(OBJEXT),%,$@),$(BINDIR))

# Memory manager for the flat build

$(BIN):	$(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

# Memory manager for the user phase of the two-pass kernel build

ifneq ($(BIN),$(UBIN))
$(UBIN):
	$(Q) $(MAKE) $(UBIN) BIN=$(UBIN) BINDIR=ubin TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
endif

# Memory manager for the kernel phase of the two-pass kernel build

ifneq ($(BIN),$(KBIN))
$(KBIN):
	$(Q) $(MAKE) $(KBIN) BIN=$(KBIN) BINDIR=kbin TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
endif

# Dependencies

.depend: Makefile $(SRCS)
ifeq ($(CONFIG_NUTTX_KERNEL),y)
	$(Q) $(MKDEP) --obj-path ubin --obj-suffix $(OBJEXT) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make_ubin.dep
	$(Q) $(MKDEP) --obj-path kbin --obj-suffix $(OBJEXT) "$(CC)" -- $(CFLAGS) $(KDEFINE) -- $(SRCS) >Make_kbin.dep
else
	$(Q) $(MKDEP) --obj-path bin --obj-suffix $(OBJEXT) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make_bin.dep
endif
	$(Q) touch $@

depend: .depend

# Clean most derived files, retaining the configuration

clean:
	$(Q) $(MAKE) -C bin  clean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C ubin clean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C kbin clean TOPDIR=$(TOPDIR)
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(UBIN))
	$(call DELFILE, $(KBIN))
	$(call CLEAN)

# Deep clean -- removes all traces of the configuration

distclean: clean
	$(Q) $(MAKE) -C bin  distclean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C ubin distclean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C kbin distclean TOPDIR=$(TOPDIR)
	$(call DELFILE, Make_bin.dep)
	$(call DELFILE, Make_ubin.dep)
	$(call DELFILE, Make_kbin.dep)
	$(call DELFILE, .depend)

-include Make_bin.dep
-include Make_ubin.dep
-include Make_kbin.dep

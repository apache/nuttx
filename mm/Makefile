############################################################################
# mm/Makefile
#
#   Copyright (C) 2007, 2012, 2013 Gregory Nutt. All rights reserved.
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

# Core allocator logic

ASRCS  = 
CSRCS  = mm_initialize.c mm_sem.c  mm_addfreechunk.c mm_size2ndx.c
CSRCS += mm_shrinkchunk.c mm_malloc.c mm_zalloc.c mm_calloc.c mm_realloc.c
CSRCS += mm_memalign.c mm_free.c mm_mallinfo.c

# Allocator instances

CSRCS += mm_user.c
ifeq ($(CONFIG_NUTTX_KERNEL),y)
CSRCS += mm_kerneluser.c
ifeq ($(CONFIG_NUTTX_KERNEL),y)
CSRCS += mm_kernel.c
endif
endif

# An optional granule allocator

ifeq ($(CONFIG_GRAN),y)
CSRCS += mm_graninit.c mm_granalloc.c mm_granfree.c mm_grancritical.c
endif

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

UBIN = libumm$(LIBEXT)
KBIN = libkmm$(LIBEXT)
BIN = libmm$(LIBEXT)

all:	$(BIN)
.PHONY: kclean uclean clean distclean 

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN):	$(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

ifneq ($(BIN),$(UBIN))
$(UBIN): kclean
	$(Q) $(MAKE) $(UBIN) BIN=$(UBIN) TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	$(Q) touch .userlib
endif

ifneq ($(BIN),$(KBIN))
$(KBIN): uclean
	$(Q) $(MAKE) $(KBIN) BIN=$(KBIN) TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	$(Q) touch .kernlib
endif

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

# Clean Targets:
# Clean user-mode temporary files (retaining the UBIN binary)

uclean:
ifneq ($(OBJEXT),)
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) if exist .userlib ]; then del *$(OBJEXT)
else
	$(Q) ( if [ -f .userlib ]; then rm -f *$(OBJEXT); fi )
endif
endif
	$(call DELFILE, .userlib)

# Clean kernel-mode temporary files (retaining the KBIN binary)

kclean:
ifneq ($(OBJEXT),)
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) if exist .kernlib ]; then del *$(OBJEXT)
else
	$(Q) ( if [ -f .kernlib ]; then rm -f *$(OBJEXT); fi )
endif
endif
	$(call DELFILE, .kernlib)

clean:
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(UBIN))
	$(call DELFILE, $(KBIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

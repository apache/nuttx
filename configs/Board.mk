############################################################################
# configs/Board.mk
#
#   Copyright (C) 2015 Gregory Nutt. All rights reserved.
#   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#           Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

ifneq ($(ZDSVERSION),)
AOBJS = $(ASRCS:.S=$(OBJEXT))
else
AOBJS = $(ASRCS:$(ASMEXT)=$(OBJEXT))
endif
COBJS = $(CSRCS:.c=$(OBJEXT))
CXXOBJS = $(CXXSRCS:.cxx=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

SCHEDSRCDIR = $(TOPDIR)$(DELIM)sched
ARCHSRCDIR = $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src
ifneq ($(CONFIG_ARCH_FAMILY),)
  ARCH_FAMILY = $(patsubst "%",%,$(CONFIG_ARCH_FAMILY))
endif

ifneq ($(ZDSVERSION),)
ifeq ($(WINTOOL),y)
  WSCHEDSRCDIR = ${shell cygpath -w $(SCHEDSRCDIR)}
  WARCHSRCDIR = ${shell cygpath -w $(ARCHSRCDIR)}
  USRINCLUDES = -usrinc:'.;$(WSCHEDSRCDIR);$(WARCHSRCDIR);$(WARCHSRCDIR)\common'
else
  USRINCLUDES = -usrinc:".;$(SCHEDSRCDIR);$(ARCHSRCDIR);$(ARCHSRCDIR)\common"
endif
else
ifeq ($(WINTOOL),y)
  CFLAGS += -I "${shell cygpath -w $(SCHEDSRCDIR)}"
ifeq ($(CONFIG_ARCH_SIM),y)
  CFLAGS += -I "${shell cygpath -w $(ARCHSRCDIR)}"
else
  CFLAGS += -I "${shell cygpath -w $(ARCHSRCDIR)$(DELIM)chip}"
  CFLAGS += -I "${shell cygpath -w $(ARCHSRCDIR)$(DELIM)common}"
ifneq ($(ARCH_FAMILY),)
  CFLAGS += -I "${shell cygpath -w $(ARCHSRCDIR)$(DELIM)$(ARCH_FAMILY)}"
endif
endif
else
  CFLAGS += -I$(SCHEDSRCDIR)
ifeq ($(CONFIG_ARCH_SIM),y)
  CFLAGS += -I$(ARCHSRCDIR)
else
  CFLAGS += -I$(ARCHSRCDIR)$(DELIM)chip
  CFLAGS += -I$(ARCHSRCDIR)$(DELIM)common
ifneq ($(ARCH_FAMILY),)
  CFLAGS += -I$(ARCHSRCDIR)$(DELIM)$(ARCH_FAMILY)
endif
endif
endif
endif

ifneq ($(ZDSVERSION),)
INCLUDES = $(ARCHSTDINCLUDES) $(USRINCLUDES)
CFLAGS = $(ARCHWARNINGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(INCLUDES) $(ARCHDEFINES) $(EXTRADEFINES)
endif

all: libboard$(LIBEXT)

ifneq ($(ZDSVERSION),)
$(ASRCS) $(HEAD_ASRC): %$(ASMEXT): %.S
ifeq ($(WINTOOL),y)
	$(Q) $(CPP) $(CPPFLAGS) `cygpath -w $<` -o $@.tmp
else
	$(Q) $(CPP) $(CPPFLAGS) $< -o $@.tmp
endif
	$(Q) cat $@.tmp | sed -e "s/^#/;/g" > $@
	$(Q) rm $@.tmp
endif

$(AOBJS): %$(OBJEXT): %$(ASMEXT)
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(LINKOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(CXXOBJS) $(LINKOBJS): %$(OBJEXT): %.cxx
	$(call COMPILEXX, $<, $@)

libboard$(LIBEXT): $(OBJS) $(CXXOBJS)
	$(Q) $(AR) $@
ifneq ($(OBJS),)
	$(call ARCHIVE, $@, $(OBJS) $(CXXOBJS))
endif

.depend: Makefile $(SRCS) $(CXXSRCS)
ifneq ($(ZDSVERSION),)
	$(Q) $(MKDEP) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
else
	$(Q) $(MKDEP) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
endif
ifneq ($(CXXSRCS),)
	$(Q) $(MKDEP) "$(CXX)" -- $(CXXFLAGS) -- $(CXXSRCS) >>Make.dep
endif
	$(Q) touch $@

depend: .depend

ifneq ($(BOARD_CONTEXT),y)
context:
endif

clean:
	$(call DELFILE, libboard$(LIBEXT))
	$(call CLEAN)
	$(EXTRA_CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)
	$(EXTRA_DISTCLEAN)

-include Make.dep

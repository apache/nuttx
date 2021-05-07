############################################################################
# boards/Board.mk
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

ifneq ($(RCSRCS)$(RCRAWS),)
ETCDIR := etctmp
ETCSRC := $(ETCDIR:%=%.c)

CSRCS += $(ETCSRC)

RCOBJS = $(RCSRCS:%=$(ETCDIR)$(DELIM)%)

$(RCOBJS): $(ETCDIR)$(DELIM)%: %
	$(Q) mkdir -p $(dir $@)
	$(call PREPROCESS, $<, $@)

$(ETCSRC): $(RCRAWS) $(RCOBJS)
	$(foreach raw, $(RCRAWS), \
	  $(shell rm -rf $(ETCDIR)$(DELIM)$(raw)) \
	  $(shell mkdir -p $(dir $(ETCDIR)$(DELIM)$(raw))) \
	  $(shell cp -rfp $(raw) $(ETCDIR)$(DELIM)$(raw)))
	$(Q) genromfs -f romfs.img -d $(ETCDIR)$(DELIM)$(CONFIG_NSH_ROMFSMOUNTPT) -V "$(basename $<)"
	$(Q) xxd -i romfs.img | sed -e "s/^unsigned/const unsigned/g" > $@
	$(Q) rm romfs.img
endif

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

CFLAGS += ${shell $(INCDIR) "$(CC)" "$(SCHEDSRCDIR)"}
CFLAGS += ${shell $(INCDIR) "$(CC)" "$(ARCHSRCDIR)$(DELIM)chip"}
ifneq ($(CONFIG_ARCH_SIM),y)
  CFLAGS += ${shell $(INCDIR) "$(CC)" "$(ARCHSRCDIR)$(DELIM)common"}
endif
ifneq ($(ARCH_FAMILY),)
  CFLAGS += ${shell $(INCDIR) "$(CC)" "$(ARCHSRCDIR)$(DELIM)$(ARCH_FAMILY)"}
endif

all: libboard$(LIBEXT)

ifneq ($(ZDSVERSION),)
$(ASRCS) $(HEAD_ASRC): %$(ASMEXT): %.S
ifeq ($(CONFIG_CYGWIN_WINTOOL),y)
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
	$(call ARCHIVE, $@, $(OBJS) $(CXXOBJS))

.depend: Makefile $(SRCS) $(CXXSRCS) $(RCSRCS) $(TOPDIR)$(DELIM).config
ifneq ($(ZDSVERSION),)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
else
	$(Q) $(MKDEP) $(DEPPATH) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
endif
ifneq ($(CXXSRCS),)
	$(Q) $(MKDEP) $(DEPPATH) "$(CXX)" -- $(CXXFLAGS) -- $(CXXSRCS) >>Make.dep
endif
ifneq ($(RCSRCS),)
	$(Q) $(MKDEP) $(DEPPATH) "$(CPP)" --obj-path . -- $(CPPFLAGS) -- $(RCSRCS) >>Make.dep
endif
	$(Q) touch $@

depend: .depend

context::

clean::
	$(call DELFILE, libboard$(LIBEXT))
	$(call DELFILE, $(ETCSRC))
	$(call DELDIR, $(ETCDIR))
	$(call CLEAN)

distclean:: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

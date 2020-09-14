############################################################################
# graphics/Makefile
#
#   Copyright (C) 2008-2009, 2011-2012, 2016, 2019 Gregory Nutt. All
#     rights reserved.
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

include nxglib/Make.defs
include nxbe/Make.defs
include nxmu/Make.defs
include nxterm/Make.defs
include vnc/Make.defs

CFLAGS += ${shell $(INCDIR) "$(CC)" $(TOPDIR)$(DELIM)graphics}

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = libgraphics$(LIBEXT)

all: mklibgraphics
.PHONY : context depend clean distclean mklibgraphics gensources gen1bppsources \
	 gen2bppsource gen4bppsource gen8bppsource gen16bppsource gen24bppsource \
	 gen32bppsources

gen1bppsources:
	$(Q) $(MAKE) -C nxglib -f Makefile.devblit NXGLIB_BITSPERPIXEL=1 EXTRAFLAGS="$(EXTRAFLAGS)"
ifeq ($(CONFIG_NX_RAMBACKED),y)
	$(Q) $(MAKE) -C nxglib -f Makefile.pwfb NXGLIB_BITSPERPIXEL=1 EXTRAFLAGS="$(EXTRAFLAGS)"
endif

gen2bppsource:
	$(Q) $(MAKE) -C nxglib -f Makefile.devblit NXGLIB_BITSPERPIXEL=2 EXTRAFLAGS="$(EXTRAFLAGS)"
ifeq ($(CONFIG_NX_RAMBACKED),y)
	$(Q) $(MAKE) -C nxglib -f Makefile.pwfb NXGLIB_BITSPERPIXEL=2 EXTRAFLAGS="$(EXTRAFLAGS)"
endif

gen4bppsource:
	$(Q) $(MAKE) -C nxglib -f Makefile.devblit NXGLIB_BITSPERPIXEL=4 EXTRAFLAGS="$(EXTRAFLAGS)"
ifeq ($(CONFIG_NX_RAMBACKED),y)
	$(Q) $(MAKE) -C nxglib -f Makefile.pwfb NXGLIB_BITSPERPIXEL=4 EXTRAFLAGS="$(EXTRAFLAGS)"
endif

gen8bppsource:
	$(Q) $(MAKE) -C nxglib -f Makefile.devblit NXGLIB_BITSPERPIXEL=8 EXTRAFLAGS="$(EXTRAFLAGS)"
ifeq ($(CONFIG_NX_RAMBACKED),y)
	$(Q) $(MAKE) -C nxglib -f Makefile.pwfb NXGLIB_BITSPERPIXEL=8 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NX_SWCURSOR),y)
	$(Q) $(MAKE) -C nxglib -f Makefile.cursor NXGLIB_BITSPERPIXEL=8 EXTRAFLAGS="$(EXTRAFLAGS)"
endif

gen16bppsource:
	$(Q) $(MAKE) -C nxglib -f Makefile.devblit NXGLIB_BITSPERPIXEL=16 EXTRAFLAGS="$(EXTRAFLAGS)"
ifeq ($(CONFIG_NX_RAMBACKED),y)
	$(Q) $(MAKE) -C nxglib -f Makefile.pwfb NXGLIB_BITSPERPIXEL=16 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NX_SWCURSOR),y)
	$(Q) $(MAKE) -C nxglib -f Makefile.cursor NXGLIB_BITSPERPIXEL=16 EXTRAFLAGS="$(EXTRAFLAGS)"
endif

gen24bppsource:
	$(Q) $(MAKE) -C nxglib -f Makefile.devblit NXGLIB_BITSPERPIXEL=24 EXTRAFLAGS="$(EXTRAFLAGS)"
ifeq ($(CONFIG_NX_RAMBACKED),y)
	$(Q) $(MAKE) -C nxglib -f Makefile.pwfb NXGLIB_BITSPERPIXEL=24 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NX_SWCURSOR),y)
	$(Q) $(MAKE) -C nxglib -f Makefile.cursor NXGLIB_BITSPERPIXEL=24 EXTRAFLAGS="$(EXTRAFLAGS)"
endif

gen32bppsources:
	$(Q) $(MAKE) -C nxglib -f Makefile.devblit NXGLIB_BITSPERPIXEL=32 EXTRAFLAGS="$(EXTRAFLAGS)"
ifeq ($(CONFIG_NX_RAMBACKED),y)
	$(Q) $(MAKE) -C nxglib -f Makefile.pwfb NXGLIB_BITSPERPIXEL=32 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NX_SWCURSOR),y)
	$(Q) $(MAKE) -C nxglib -f Makefile.cursor NXGLIB_BITSPERPIXEL=32 EXTRAFLAGS="$(EXTRAFLAGS)"
endif

gensources: gen1bppsources gen2bppsource gen4bppsource gen8bppsource gen16bppsource gen24bppsource gen32bppsources

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

mklibgraphics: $(BIN)

.depend: gensources Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean_context:
	$(Q) $(MAKE) -C nxglib -f Makefile.devblit distclean EXTRAFLAGS="$(EXTRAFLAGS)"
	$(Q) $(MAKE) -C nxglib -f Makefile.pwfb distclean EXTRAFLAGS="$(EXTRAFLAGS)"
	$(Q) $(MAKE) -C nxglib -f Makefile.cursor distclean EXTRAFLAGS="$(EXTRAFLAGS)"

context: gensources

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean clean_context
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

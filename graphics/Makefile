############################################################################
# graphics/Makefile
#
#   Copyright (C) 2008-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

DEPPATH		= --dep-path .

ifeq ($(WINTOOL),y)
INCDIROPT	= -w
endif

include nxglib/Make.defs
DEPPATH		+= --dep-path nxglib
CFLAGS		+= ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(TOPDIR)/graphics/nxglib}

include nxbe/Make.defs
DEPPATH		+= --dep-path nxbe
CFLAGS		+= ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(TOPDIR)/graphics/nxbe}

ifeq ($(CONFIG_NX_MULTIUSER),y)
include nxmu/Make.defs
DEPPATH		+= --dep-path nxmu
CFLAGS		+= ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(TOPDIR)/graphics/nxmu}
else
include nxsu/Make.defs
DEPPATH		+= --dep-path nxsu
CFLAGS		+= ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(TOPDIR)/graphics/nxsu}
endif

ifeq ($(CONFIG_NXTERM),y)
include nxterm/Make.defs
DEPPATH		+= --dep-path nxterm
CFLAGS		+= ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(TOPDIR)/graphics/nxterm}
endif

ASRCS		= $(NXGLIB_ASRCS) $(NXBE_ASRCS) $(NX_ASRCS) $(NXTK_ASRCS) $(NXTERM_ASRCS)
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= $(NXGLIB_CSRCS) $(NXBE_CSRCS) $(NX_CSRCS) $(NXTK_CSRCS) $(NXTERM_CSRCS)
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN		= libgraphics$(LIBEXT)

ifeq ($(CONFIG_NX_MULTIUSER),y)
VPATH		= nxglib:nxbe:nxmu:nxterm
else
VPATH		= nxglib:nxbe:nxsu:nxterm
endif

all:	mklibgraphics
.PHONY : context depend clean distclean mklibgraphics gensources gen1bppsources \
	 gen2bppsource gen4bppsource gen8bppsource gen16bppsource gen24bppsource \
	 gen32bppsources

gen1bppsources:
	$(Q) $(MAKE) -C nxglib -f Makefile.sources TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=1 EXTRADEFINES=$(EXTRADEFINES)

gen2bppsource:
	$(Q) $(MAKE) -C nxglib -f Makefile.sources TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=2 EXTRADEFINES=$(EXTRADEFINES)

gen4bppsource:
	$(Q) $(MAKE) -C nxglib -f Makefile.sources TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=4 EXTRADEFINES=$(EXTRADEFINES)

gen8bppsource:
	$(Q) $(MAKE) -C nxglib -f Makefile.sources TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=8 EXTRADEFINES=$(EXTRADEFINES)

gen16bppsource:
	$(Q) $(MAKE) -C nxglib -f Makefile.sources TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=16 EXTRADEFINES=$(EXTRADEFINES)

gen24bppsource:
	$(Q) $(MAKE) -C nxglib -f Makefile.sources TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=24 EXTRADEFINES=$(EXTRADEFINES)

gen32bppsources:
	$(Q) $(MAKE) -C nxglib -f Makefile.sources TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=32 EXTRADEFINES=$(EXTRADEFINES)

gensources: gen1bppsources gen2bppsource gen4bppsource gen8bppsource gen16bppsource gen24bppsource gen32bppsources

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

mklibgraphics: $(BIN)

.depend: gensources Makefile $(SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

context: gensources

clean:
	$(Q) $(MAKE) -C nxglib -f Makefile.sources clean TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(Q) $(MAKE) -C nxglib -f Makefile.sources distclean TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep


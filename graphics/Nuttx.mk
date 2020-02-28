############################################################################
# graphics/Nuttx.mk
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

-include $(TOPDIR)/Nuttx.defs

DEPPATH = --dep-path .

ifeq ($(WINTOOL),y)
INCDIROPT = -w
endif

ASRCS =
CSRCS =
VPATH =

include nxglib/Nuttx.defs
include nxbe/Nuttx.defs
include nxmu/Nuttx.defs
include nxterm/Nuttx.defs
include vnc/Nuttx.defs

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
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.devblit TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=1 EXTRADEFINES=$(EXTRADEFINES)
ifeq ($(CONFIG_NX_RAMBACKED),y)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.pwfb TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=1 EXTRADEFINES=$(EXTRADEFINES)
endif

gen2bppsource:
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.devblit TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=2 EXTRADEFINES=$(EXTRADEFINES)
ifeq ($(CONFIG_NX_RAMBACKED),y)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.pwfb TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=2 EXTRADEFINES=$(EXTRADEFINES)
endif

gen4bppsource:
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.devblit TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=4 EXTRADEFINES=$(EXTRADEFINES)
ifeq ($(CONFIG_NX_RAMBACKED),y)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.pwfb TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=4 EXTRADEFINES=$(EXTRADEFINES)
endif

gen8bppsource:
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.devblit TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=8 EXTRADEFINES=$(EXTRADEFINES)
ifeq ($(CONFIG_NX_RAMBACKED),y)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.pwfb TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=8 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NX_SWCURSOR),y)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.cursor TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=8 EXTRADEFINES=$(EXTRADEFINES)
endif

gen16bppsource:
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.devblit TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=16 EXTRADEFINES=$(EXTRADEFINES)
ifeq ($(CONFIG_NX_RAMBACKED),y)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.pwfb TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=16 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NX_SWCURSOR),y)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.cursor TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=16 EXTRADEFINES=$(EXTRADEFINES)
endif

gen24bppsource:
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.devblit TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=24 EXTRADEFINES=$(EXTRADEFINES)
ifeq ($(CONFIG_NX_RAMBACKED),y)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.pwfb TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=24 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NX_SWCURSOR),y)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.cursor TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=24 EXTRADEFINES=$(EXTRADEFINES)
endif

gen32bppsources:
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.devblit TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=32 EXTRADEFINES=$(EXTRADEFINES)
ifeq ($(CONFIG_NX_RAMBACKED),y)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.pwfb TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=32 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NX_SWCURSOR),y)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.cursor TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=32 EXTRADEFINES=$(EXTRADEFINES)
endif

gensources: gen1bppsources gen2bppsource gen4bppsource gen8bppsource gen16bppsource gen24bppsource gen32bppsources

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

mklibgraphics: $(BIN)

.depend: gensources Nuttx.mk $(SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Nuttx.dep
	$(Q) touch $@

depend: .depend

clean_context:
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.devblit distclean TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.pwfb distclean TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.cursor distclean TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)

context: gensources

clean:
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.devblit clean TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.pwfb clean TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	$(Q) $(MAKE) -C nxglib -f Nuttx.mk.cursor clean TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean clean_context
	$(call DELFILE, Nuttx.dep)
	$(call DELFILE, .depend)

-include Nuttx.dep

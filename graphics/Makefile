############################################################################
# graphics/Makefile
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
############################################################################

include $(TOPDIR)/Make.defs

include nxglib/Make.defs
include nxbe/Make.defs
include nxmu/Make.defs
include nxterm/Make.defs

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
	$(call ARCHIVE, $@, $^)

mklibgraphics: $(BIN)

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: gensources Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MAKE) makedepfile
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

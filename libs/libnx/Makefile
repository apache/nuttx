############################################################################
# libs/libnx/Makefile
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
###########################################################################

include $(TOPDIR)/Make.defs

include nxglib/Make.defs
include nx/Make.defs
include nxmu/Make.defs
include nxfonts/Make.defs
include nxtk/Make.defs

BINDIR ?= bin

CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)libs$(DELIM)libnx

AOBJS = $(patsubst %.S, $(BINDIR)$(DELIM)%$(OBJEXT), $(ASRCS))
COBJS = $(patsubst %.c, $(BINDIR)$(DELIM)%$(OBJEXT), $(CSRCS))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

KBIN = libknx$(LIBEXT)
BIN ?= libnx$(LIBEXT)

all: $(BIN)
.PHONY: clean distclean
.PHONY : context depend clean distclean gensources gen1bppsources gen2bppsource \
	 gen4bppsource gen8bppsource gen16bppsource gen24bppsource gen32bppsources genfontsources

gen1bppsources:
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_BITSPERPIXEL=1 EXTRAFLAGS="$(EXTRAFLAGS)"

gen2bppsource:
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_BITSPERPIXEL=2 EXTRAFLAGS="$(EXTRAFLAGS)"

gen4bppsource:
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_BITSPERPIXEL=4 EXTRAFLAGS="$(EXTRAFLAGS)"

gen8bppsource:
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_BITSPERPIXEL=8 EXTRAFLAGS="$(EXTRAFLAGS)"

gen16bppsource:
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_BITSPERPIXEL=16 EXTRAFLAGS="$(EXTRAFLAGS)"

gen24bppsource:
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_BITSPERPIXEL=24 EXTRAFLAGS="$(EXTRAFLAGS)"

gen32bppsources:
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_BITSPERPIXEL=32 EXTRAFLAGS="$(EXTRAFLAGS)"

genfontsources:
ifeq ($(CONFIG_NXFONT_MONO5X8),y)
	@$(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=18 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SANS23X27),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=1 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SANS22X29),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=2 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SANS28X37),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=3 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SANS39X48),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=4 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SANS17X23B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=16 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SANS20X27B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=17 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SANS22X29B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=5 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SANS28X37B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=6 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SANS40X49B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=7 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SERIF22X29),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=8 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SERIF29X37),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=9 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SERIF38X48),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=10 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SERIF22X28B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=11 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SERIF27X38B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=12 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SERIF38X49B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=13 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SANS17X22),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=14 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_SANS20X26),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=15 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_PIXEL_UNICODE),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=19 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_PIXEL_LCD_MACHINE),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=20 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_4X6),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=21 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_5X7),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=22 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_5X8),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=23 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_6X9),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=24 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_6X10),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=25 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_6X12),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=26 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_6X13),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=27 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_6X13B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=28 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_6X13O),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=29 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_7X13),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=30 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_7X13B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=31 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_7X13O),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=32 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_7X14),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=33 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_7X14B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=34 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_8X13),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=35 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_8X13B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=36 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_8X13O),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=37 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_9X15),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=38 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_9X15B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=39 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_9X18),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=40 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_9X18B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=41 EXTRAFLAGS="$(EXTRAFLAGS)"
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_10X20),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=42 EXTRAFLAGS="$(EXTRAFLAGS)"
endif

ifeq ($(CONFIG_NXFONT_TOM_THUMB_4X6),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources NXFONTS_FONTID=43 EXTRAFLAGS="$(EXTRAFLAGS)"
endif

gensources: gen1bppsources gen2bppsource gen4bppsource gen8bppsource gen16bppsource gen24bppsource gen32bppsources genfontsources

$(AOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

# NX library for the flat build and
# the user phase of the two-pass kernel build

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

# NX library for the kernel phase of the two-pass kernel build

ifneq ($(BIN),$(KBIN))
$(KBIN): $(OBJS)
	$(Q) $(MAKE) $(KBIN) BIN=$(KBIN) EXTRAFLAGS="$(EXTRAFLAGS)"
endif

# Dependencies

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, bin/Make.dep, $^)
	$(call DELFILE, $^)

makekdepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, kbin/Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile gensources $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MAKE) makedepfile OBJPATH="bin"
ifneq ($(CONFIG_BUILD_FLAT),y)
	$(Q) $(MAKE) makekdepfile CFLAGS="$(CFLAGS) $(KDEFINE)" OBJPATH="kbin"
endif
	$(Q) touch $@

depend: .depend

# Generate configuration context

context: gensources

# Clean most derived files, retaining the configuration

clean:
	$(Q) $(MAKE) -C bin  clean
	$(Q) $(MAKE) -C kbin clean
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources clean EXTRAFLAGS="$(EXTRAFLAGS)"
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(KBIN))
	$(call CLEAN)

# Deep clean -- removes all traces of the configuration

distclean: clean
	$(Q) $(MAKE) -C bin  distclean
	$(Q) $(MAKE) -C kbin distclean
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources distclean EXTRAFLAGS="$(EXTRAFLAGS)"
	$(call DELFILE, bin/Make.dep)
	$(call DELFILE, kbin/Make.dep)
	$(call DELFILE, .depend)

-include bin/Make.dep
-include kbin/Make.dep

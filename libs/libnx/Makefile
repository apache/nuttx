############################################################################
# libs/libnx/Makefile
#
#   Copyright (C) 2013, 2017 Gregory Nutt. All rights reserved.
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
###########################################################################

-include $(TOPDIR)/Make.defs

# CFLAGS

ifneq ($(CONFIG_BUILD_FLAT),y)
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  KDEFINE = ${shell $(TOPDIR)\tools\define.bat "$(CC)" __KERNEL__}
else
  KDEFINE = ${shell $(TOPDIR)/tools/define.sh "$(CC)" __KERNEL__}
endif
endif

# Sources and paths

ASRCS =
CSRCS =

DEPPATH := --dep-path .
VPATH := .

include nxglib/Make.defs
include nx/Make.defs
include nxmu/Make.defs
include nxfonts/Make.defs
include nxtk/Make.defs

BINDIR ?= bin

AOBJS = $(patsubst %.S, $(BINDIR)$(DELIM)%$(OBJEXT), $(ASRCS))
COBJS = $(patsubst %.c, $(BINDIR)$(DELIM)%$(OBJEXT), $(CSRCS))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

UBIN = libunx$(LIBEXT)
KBIN = libknx$(LIBEXT)
BIN ?= libnx$(LIBEXT)

all: $(BIN)
.PHONY: clean distclean
.PHONY : context depend clean distclean gensources gen1bppsources gen2bppsource \
	 gen4bppsource gen8bppsource gen16bppsource gen24bppsource gen32bppsources genfontsources

gen1bppsources:
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_BITSPERPIXEL=1 EXTRADEFINES=$(EXTRADEFINES)

gen2bppsource:
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_BITSPERPIXEL=2 EXTRADEFINES=$(EXTRADEFINES)

gen4bppsource:
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_BITSPERPIXEL=4 EXTRADEFINES=$(EXTRADEFINES)

gen8bppsource:
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_BITSPERPIXEL=8 EXTRADEFINES=$(EXTRADEFINES)

gen16bppsource:
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_BITSPERPIXEL=16 EXTRADEFINES=$(EXTRADEFINES)

gen24bppsource:
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_BITSPERPIXEL=24 EXTRADEFINES=$(EXTRADEFINES)

gen32bppsources:
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_BITSPERPIXEL=32 EXTRADEFINES=$(EXTRADEFINES)

genfontsources:
ifeq ($(CONFIG_NXFONT_MONO5X8),y)
	@$(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=18 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SANS23X27),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=1 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SANS22X29),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=2 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SANS28X37),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=3 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SANS39X48),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=4 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SANS17X23B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=16 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SANS20X27B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=17 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SANS22X29B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=5 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SANS28X37B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=6 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SANS40X49B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=7 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SERIF22X29),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=8 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SERIF29X37),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=9 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SERIF38X48),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=10 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SERIF22X28B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=11 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SERIF27X38B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=12 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SERIF38X49B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=13 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SANS17X22),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=14 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_SANS20X26),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=15 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_PIXEL_UNICODE),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=19 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_PIXEL_LCD_MACHINE),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=20 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_4X6),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=21 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_5X7),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=22 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_5X8),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=23 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_6X9),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=24 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_6X10),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=25 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_6X12),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=26 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_6X13),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=27 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_6X13B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=28 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_6X13O),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=29 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_7X13),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=30 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_7X13B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=31 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_7X13O),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=32 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_7X14),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=33 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_7X14B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=34 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_8X13),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=35 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_8X13B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=36 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_8X13O),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=37 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_9X15),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=38 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_9X15B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=39 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_9X18),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=40 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_9X18B),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=41 EXTRADEFINES=$(EXTRADEFINES)
endif
ifeq ($(CONFIG_NXFONT_X11_MISC_FIXED_10X20),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=42 EXTRADEFINES=$(EXTRADEFINES)
endif

ifeq ($(CONFIG_NXFONT_TOM_THUMB_4X6),y)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=43 EXTRADEFINES=$(EXTRADEFINES)
endif

gensources: gen1bppsources gen2bppsource gen4bppsource gen8bppsource gen16bppsource gen24bppsource gen32bppsources genfontsources

$(AOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)
	$(call MOVEOBJ,$(patsubst $(BINDIR)$(DELIM)%$(OBJEXT),%,$@),$(BINDIR))

$(COBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.c
	$(call COMPILE, $<, $@)
	$(call MOVEOBJ,$(patsubst $(BINDIR)$(DELIM)%$(OBJEXT),%,$@),$(BINDIR))

# NX library for the flat build

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

# NX library for the user phase of the two-pass kernel build

ifneq ($(BIN),$(UBIN))
$(UBIN):
	$(Q) $(MAKE) $(UBIN) BIN=$(UBIN) BINDIR=ubin TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
endif

# NX library for the kernel phase of the two-pass kernel build

ifneq ($(BIN),$(KBIN))
$(KBIN):
	$(Q) $(MAKE) $(KBIN) BIN=$(KBIN) BINDIR=kbin TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
endif

# Dependencies

.depend: Makefile gensources $(SRCS)
ifeq ($(CONFIG_BUILD_FLAT),y)
	$(Q) $(MKDEP) --obj-path bin --obj-suffix $(OBJEXT) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make_bin.dep
else
	$(Q) $(MKDEP) --obj-path ubin --obj-suffix $(OBJEXT) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make_ubin.dep
	$(Q) $(MKDEP) --obj-path kbin --obj-suffix $(OBJEXT) $(DEPPATH) "$(CC)" -- $(CFLAGS) $(KDEFINE) -- $(SRCS) >Make_kbin.dep
endif
	$(Q) touch $@

depend: .depend

# Generate configuration context

context: gensources

# Clean most derived files, retaining the configuration

clean:
	$(Q) $(MAKE) -C bin  clean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C ubin clean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C kbin clean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources clean TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(UBIN))
	$(call DELFILE, $(KBIN))
	$(call CLEAN)

# Deep clean -- removes all traces of the configuration

distclean: clean
	$(Q) $(MAKE) -C bin  distclean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C ubin distclean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C kbin distclean TOPDIR=$(TOPDIR)
	$(Q) $(MAKE) -C nxfonts -f Makefile.sources distclean TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	$(call DELFILE, Make_bin.dep)
	$(call DELFILE, Make_ubin.dep)
	$(call DELFILE, Make_kbin.dep)
	$(call DELFILE, .depend)

-include Make_bin.dep
-include Make_ubin.dep
-include Make_kbin.dep

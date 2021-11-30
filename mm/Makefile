############################################################################
# mm/Makefile
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

# Sources and paths

include mm_heap/Make.defs
include umm_heap/Make.defs
include kmm_heap/Make.defs
include mm_gran/Make.defs
include shm/Make.defs
include iob/Make.defs
include circbuf/Make.defs
include kasan/Make.defs

BINDIR ?= bin

CFLAGS += ${shell $(INCDIR) "$(CC)" $(TOPDIR)$(DELIM)mm}

AOBJS = $(patsubst %.S, $(BINDIR)$(DELIM)%$(OBJEXT), $(ASRCS))
COBJS = $(patsubst %.c, $(BINDIR)$(DELIM)%$(OBJEXT), $(CSRCS))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

KBIN = libkmm$(LIBEXT)
BIN ?= libmm$(LIBEXT)

all: $(BIN)
.PHONY: clean distclean

$(AOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

# Memory manager for the flat build and
# the user phase of the two-pass kernel build

$(BIN):	$(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

# Memory manager for the kernel phase of the two-pass kernel build

ifneq ($(BIN),$(KBIN))
$(KBIN):
	$(Q) $(MAKE) $(KBIN) BIN=$(KBIN) BINDIR=kbin EXTRAFLAGS="$(EXTRAFLAGS)"
endif

# Dependencies

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, bin/Make.dep, $^)
	$(call DELFILE, $^)

makekdepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, kbin/Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MAKE) makedepfile OBJPATH="bin"
ifneq ($(CONFIG_BUILD_FLAT),y)
	$(Q) $(MAKE) makekdepfile CFLAGS="$(CFLAGS) $(KDEFINE)" OBJPATH="kbin"
endif
	$(Q) touch $@

depend: .depend

# Clean most derived files, retaining the configuration

clean:
	$(Q) $(MAKE) -C bin  clean
	$(Q) $(MAKE) -C kbin clean
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(KBIN))
	$(call CLEAN)

# Deep clean -- removes all traces of the configuration

distclean: clean
	$(Q) $(MAKE) -C bin  distclean
	$(Q) $(MAKE) -C kbin distclean
	$(call DELFILE, bin/Make.dep)
	$(call DELFILE, kbin/Make.dep)
	$(call DELFILE, .depend)

-include bin/Make.dep
-include kbin/Make.dep

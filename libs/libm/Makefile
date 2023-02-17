############################################################################
# libs/libc/Makefile
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

ifeq ($(CONFIG_LIBM),y)
include libm/Make.defs
endif

BINDIR ?= bin

AOBJS = $(patsubst %.S, $(BINDIR)$(DELIM)$(DELIM)%$(OBJEXT), $(ASRCS))
COBJS = $(patsubst %.c, $(BINDIR)$(DELIM)$(DELIM)%$(OBJEXT), $(CSRCS))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

KBIN = libkm$(LIBEXT)
BIN ?= libm$(LIBEXT)

all: $(BIN)
.PHONY: clean distclean

$(AOBJS): $(BINDIR)$(DELIM)$(DELIM)%$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): $(BINDIR)$(DELIM)$(DELIM)%$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

# math library for the flat build and
# the user phase of the two-pass kernel build

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

# math library for the kernel phase of the two-pass kernel build

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
	$(call DELFILE, bin$(DELIM)Make.dep)
	$(call DELFILE, kbin$(DELIM)Make.dep)
	$(call DELFILE, .depend)

-include bin$(DELIM)Make.dep
-include kbin$(DELIM)Make.dep

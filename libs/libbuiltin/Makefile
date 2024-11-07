############################################################################
# libs/libbuiltin/Makefile
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

BIN ?= libbuiltin$(LIBEXT)
BINDIR ?= bin

KBIN = libkbuiltin$(LIBEXT)
KBINDIR = kbin

include compiler-rt/Make.defs

include libgcc/Make.defs

AOBJS = $(addprefix $(BINDIR)$(DELIM), $(ASRCS:.S=$(OBJEXT)))
COBJS = $(addprefix $(BINDIR)$(DELIM), $(CSRCS:.c=$(OBJEXT)))
CXXOBJS = $(addprefix $(BINDIR)$(DELIM), $(CXXSRCS:.cxx=$(OBJEXT)))
CPPOBJS = $(addprefix $(BINDIR)$(DELIM), $(CPPSRCS:.cpp=$(OBJEXT)))

SRCS = $(ASRCS) $(CSRCS) $(CXXSRCS) $(CPPSRCS)
OBJS = $(AOBJS) $(COBJS) $(CXXOBJS) $(CPPOBJS)

all: $(BIN)
.PHONY: depend clean distclean context

$(AOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(CXXOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.cxx
	$(call COMPILEXX, $<, $@)

$(CPPOBJS): $(BINDIR)$(DELIM)%$(OBJEXT): %.cpp
	$(call COMPILEXX, $<, $@)

bin:
	$(Q) mkdir $@

kbin:
	$(Q) mkdir $@

context:: bin kbin

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) touch $@

depend:: .depend

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

# C library for the kernel phase of the two-pass kernel build

ifneq ($(BIN),$(KBIN))
$(KBIN): $(OBJS)
	$(Q) $(MAKE) $(KBIN) BIN=$(KBIN) BINDIR=$(KBINDIR) EXTRAFLAGS="$(EXTRAFLAGS)"
endif

clean:
	$(call CLEAN)
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(KBIN))

distclean:: clean
	$(call DELDIR, bin)
	$(call DELDIR, kbin)
	$(call DELDIR, .depend)

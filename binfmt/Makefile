############################################################################
# binfmt/Makefile
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

# Basic BINFMT source files

CSRCS  = binfmt_globals.c binfmt_initialize.c binfmt_register.c binfmt_unregister.c
CSRCS += binfmt_loadmodule.c binfmt_unloadmodule.c binfmt_execmodule.c
CSRCS += binfmt_exec.c binfmt_copyargv.c binfmt_dumpmodule.c
CSRCS += binfmt_coredump.c

ifeq ($(CONFIG_BINFMT_LOADABLE),y)
CSRCS += binfmt_exit.c
endif

ifeq ($(CONFIG_LIBC_EXECFUNCS),y)
CSRCS += binfmt_execsymtab.c
endif

# Builtin application interfaces

ifeq ($(CONFIG_BUILTIN),y)
CSRCS += builtin.c
endif

# Add configured binary modules

include libnxflat/Make.defs
include libelf/Make.defs

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN = libbinfmt$(LIBEXT)

all: $(BIN)
.PHONY: depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
	$(Q) $(MAKE) makedepfile
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

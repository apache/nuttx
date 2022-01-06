############################################################################
# arch/risc-v/src/Makefile
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
include chip/Make.defs

ifeq ($(CONFIG_ARCH_RV32I),y)           # Base Integer support
ARCH_SUBDIR = rv32i
else ifeq ($(CONFIG_ARCH_RV32IM),y)     # Integer + math support
ARCH_SUBDIR = rv32im
else ifeq ($(CONFIG_ARCH_RV64GC),y)     # GC=IMAFDC
ARCH_SUBDIR = rv64gc
else
ARCH_SUBDIR = rv32i                     # Default to base RV32I core
endif

ARCH_SRCDIR = $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src

INCLUDES += ${shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)chip}
INCLUDES += ${shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)common}
INCLUDES += ${shell $(INCDIR) "$(CC)" $(ARCH_SRCDIR)$(DELIM)$(ARCH_SUBDIR)}
INCLUDES += ${shell $(INCDIR) "$(CC)" $(TOPDIR)$(DELIM)sched}

CPPFLAGS += $(INCLUDES)
CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)
AFLAGS += $(INCLUDES)

ifeq ($(CONFIG_CYGWIN_WINTOOL),y)
  NUTTX = "${shell cygpath -w $(TOPDIR)$(DELIM)nuttx$(EXEEXT)}"
else
  NUTTX = "$(TOPDIR)$(DELIM)nuttx$(EXEEXT)"
endif

# Additional rules for system call wrapper

ifeq ($(CONFIG_SCHED_INSTRUMENTATION_SYSCALL),y)
  EXTRALINKCMDS += @$(TOPDIR)/syscall/syscall_wraps.ldcmd
endif

# The "head" object

HEAD_OBJ = $(HEAD_ASRC:.S=$(OBJEXT))
STARTUP_OBJS ?= $(HEAD_OBJ)

# Flat build or kernel-mode objects

ASRCS = $(CHIP_ASRCS) $(CMN_ASRCS)
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = $(CHIP_CSRCS) $(CMN_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

# User-mode objects

UASRCS = $(CHIP_UASRCS) $(CMN_UASRCS)
UAOBJS = $(UASRCS:.S=$(OBJEXT))

UCSRCS = $(CHIP_UCSRCS) $(CMN_UCSRCS)
UCOBJS = $(UCSRCS:.c=$(OBJEXT))

USRCS = $(UASRCS) $(UCSRCS)
UOBJS = $(UAOBJS) $(UCOBJS)

KBIN = libkarch$(LIBEXT)
BIN  = libarch$(LIBEXT)

LDFLAGS += $(ARCHSCRIPT) $(EXTRALINKCMDS)

LDSTARTGROUP ?= --start-group
LDENDGROUP ?= --end-group

BOARDMAKE = $(if $(wildcard board$(DELIM)Makefile),y,)

ifeq ($(CONFIG_CYGWIN_WINTOOL),y)
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)$(DELIM)staging"}"
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)board"}"
endif

else
  LIBPATHS += -L"$(TOPDIR)$(DELIM)staging"
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -L"$(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)board"
endif

endif

LDLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))
ifeq ($(BOARDMAKE),y)
  LDLIBS += -lboard
endif

VPATH += chip
VPATH += common
VPATH += $(ARCH_SUBDIR)
VPATH += $(CHIP_DIR)

all: $(HEAD_OBJ) $(BIN)

.PHONY: board/libboard$(LIBEXT)

$(AOBJS) $(UAOBJS) $(HEAD_OBJ): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(UCOBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

ifeq ($(CONFIG_BUILD_FLAT),y)
$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))
else
$(BIN): $(UOBJS)
	$(call ARCHIVE, $@, $(UOBJS))
endif

$(KBIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

board/libboard$(LIBEXT):
	$(Q) $(MAKE) -C board libboard$(LIBEXT) EXTRAFLAGS="$(EXTRAFLAGS)"

nuttx$(EXEEXT): $(HEAD_OBJ) board/libboard$(LIBEXT)
	$(Q) echo "LD: nuttx"
	$(Q) $(LD) --entry=__start $(LDFLAGS) $(LIBPATHS) $(EXTRA_LIBPATHS) \
		-o $(NUTTX) $(HEAD_OBJ) $(EXTRA_OBJS) \
		$(LDSTARTGROUP) $(LDLIBS) $(EXTRA_LIBS) $(LDENDGROUP)
ifneq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) $(NM) $(NUTTX) | \
	grep -v '\(compiled\)\|\(\$(OBJEXT)$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
	sort > $(TOPDIR)/System.map
endif

# This is part of the top-level export target
# Note that there may not be a head object if layout is handled
# by the linker configuration.

export_startup: board/libboard$(LIBEXT) $(STARTUP_OBJS)
ifneq ($(STARTUP_OBJS),)
	$(Q) if [ -d "$(EXPORT_DIR)/startup" ]; then \
		cp -f $(STARTUP_OBJS) "$(EXPORT_DIR)/startup/."; \
	 else \
		echo "$(EXPORT_DIR)/startup does not exist"; \
	exit 1; \
	fi
endif

# Dependencies

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds) $(HEAD_ASRC:.S=.dds)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile $(SRCS) $(TOPDIR)$(DELIM).config
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board depend
endif
	$(Q) $(MAKE) makedepfile DEPPATH="$(patsubst %,--dep-path %,$(subst :, ,$(VPATH)))"
	$(Q) touch $@

depend: .depend

context::

clean:
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board clean
endif
	$(call DELFILE, $(KBIN))
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(HEAD_OBJ))
	$(call CLEAN)

distclean:: clean
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board distclean
endif
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

info:
	@echo $(HEAD_OBJ)
	@echo $(ASRCS)
	@echo $(CONFIG_ARCH_CHIP)

-include Make.dep

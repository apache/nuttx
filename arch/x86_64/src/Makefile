############################################################################
# arch/x86_64/src/Makefile
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

ifeq ($(CONFIG_ARCH_INTEL64),y)
ARCH_SUBDIR = intel64
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

ASRCS = $(CHIP_ASRCS) $(CMN_ASRCS)
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = $(CHIP_CSRCS) $(CMN_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

KBIN = libkarch$(LIBEXT)
BIN  = libarch$(LIBEXT)

# Override in Make.defs if linker is not 'ld'

LDSTARTGROUP ?= --start-group
LDENDGROUP ?= --end-group

LDFLAGS += -z max-page-size=0x1000 $(ARCHSCRIPT)
LOADER_LDFLAGS += $(LOADER_ARCHSCRIPT)

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

# Add the builtin library

EXTRA_LIBS += -lgcc
EXTRA_LIBPATHS += -L "${dir ${shell $(CC) $(ARCHCPUFLAGS) --print-libgcc-file-name}}"

ifneq ($(CONFIG_LIBM),y)
  EXTRA_LIBS += -lm
  EXTRA_LIBPATHS += -L "${dir ${shell $(CC) $(ARCHCPUFLAGS) --print-file-name=libm.a}}"
endif

ifeq ($(CONFIG_CXX_LIBSUPCXX),y)
  EXTRA_LIBS += -lsupc++
  EXTRA_LIBPATHS += -L "${dir ${shell $(CC) $(ARCHCPUFLAGS) --print-file-name=libsupc++.a}}"
endif

VPATH = chip:common:$(ARCH_SUBDIR)

all: libarch$(LIBEXT)

.PHONY: board/libboard$(LIBEXT)

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
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

nuttx$(EXEEXT): board/libboard$(LIBEXT)
	@echo "LD: nuttx$(EXEEXT)"
	$(Q) $(LD) $(LDFLAGS) $(LIBPATHS) $(EXTRA_LIBPATHS) \
		-o $(NUTTX) $(EXTRA_OBJS) \
		$(LDSTARTGROUP) $(EXTRA_LIBS) --whole-archive --no-relax $(LDLIBS) $(LDENDGROUP)
ifneq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) $(NM) $(NUTTX) | \
	grep -v '\(compiled\)\|\(\$(OBJEXT)$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
	sort > $(TOPDIR)/System.map
endif

# This is part of the top-level export target

export_startup: board/libboard$(LIBEXT) $(STARTUP_OBJS)
	$(Q) if [ -d "$(EXPORT_DIR)/startup" ]; then \
		cp -f $(STARTUP_OBJS) "$(EXPORT_DIR)/startup"; \
	 else \
		echo "$(EXPORT_DIR)/startup does not exist"; \
		exit 1; \
	 fi

# Dependencies

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile chip/Make.defs $(SRCS) $(TOPDIR)$(DELIM).config
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board depend
endif
	$(Q) $(MAKE) makedepfile DEPPATH="--dep-path chip --dep-path common --dep-path $(ARCH_SUBDIR)"
	$(Q) touch $@

depend: .depend

context::

clean:
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board clean
endif
	$(call DELFILE, libarch$(LIBEXT))
	$(call CLEAN)

distclean:: clean
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board distclean
endif
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

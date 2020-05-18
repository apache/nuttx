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

-include $(TOPDIR)/Make.defs
-include chip/Make.defs

ifeq ($(CONFIG_ARCH_INTEL64),y)
ARCH_SUBDIR = intel64
endif

CPPFLAGS += $(EXTRAFLAGS)
CFLAGS += $(EXTRAFLAGS)
CXXFLAGS += $(EXTRAFLAGS)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  ARCH_SRCDIR = $(TOPDIR)\arch\$(CONFIG_ARCH)\src
  NUTTX = $(TOPDIR)\nuttx$(EXEEXT)
  CFLAGS += -I$(ARCH_SRCDIR)\chip
  CFLAGS += -I$(ARCH_SRCDIR)\common
  CFLAGS += -I$(ARCH_SRCDIR)\$(ARCH_SUBDIR)
  CFLAGS += -I$(TOPDIR)\sched
else
  ARCH_SRCDIR = $(TOPDIR)/arch/$(CONFIG_ARCH)/src
ifeq ($(CONFIG_CYGWIN_WINTOOL),y)
  NUTTX = "${shell cygpath -w $(TOPDIR)/nuttx$(EXEEXT)}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/chip}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/common}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/$(ARCH_SUBDIR)}"
  CFLAGS += -I "${shell cygpath -w $(TOPDIR)/sched}"
else
  NUTTX = $(TOPDIR)/nuttx$(EXEEXT)
  LOADER = $(TOPDIR)/loader$(EXEEXT)
  CFLAGS += -I$(ARCH_SRCDIR)/chip
  CFLAGS += -I$(ARCH_SRCDIR)/common
  CFLAGS += -I$(ARCH_SRCDIR)/$(ARCH_SUBDIR)
  CFLAGS += -I$(TOPDIR)/sched
endif
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
EXTRA_LIBS ?=
LINKLIBS ?=

OBJCOPYARGS =

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BOARDMAKE = $(if $(wildcard .\board\Makefile),y,)
  LIBPATHS += -L"$(TOPDIR)\staging"
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -L"$(TOPDIR)\arch\$(CONFIG_ARCH)\src\board"
endif

else
  BOARDMAKE = $(if $(wildcard ./board/Makefile),y,)

ifeq ($(CONFIG_CYGWIN_WINTOOL),y)
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)/staging"}"
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)/arch/$(CONFIG_ARCH)/src/board"}"
endif

else
  LIBPATHS += -L"$(TOPDIR)/staging"
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -L"$(TOPDIR)/arch/$(CONFIG_ARCH)/src/board"
endif
endif
endif

LDLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))
ifeq ($(BOARDMAKE),y)
  LDLIBS += -lboard
endif

LIBGCC = "${shell "$(CC)" $(ARCHCPUFLAGS) -print-libgcc-file-name}"

ifeq ($(HOSTOS),FreeBSD)
  HOST_ARCH = ${shell uname -m 2>/dev/null || echo "Other"}
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
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" libboard$(LIBEXT) EXTRAFLAGS="$(EXTRAFLAGS)"

nuttx$(EXEEXT): board/libboard$(LIBEXT)
	@echo "LD: nuttx$(EXEEXT)"
	$(Q) $(LD) $(LDFLAGS) $(LIBPATHS) -o $(NUTTX) $(EXTRA_OBJS) \
		$(LDSTARTGROUP) $(LIBGCC) --whole-archive --no-relax  $(LDLIBS) $(EXTRA_LIBS) $(LDENDGROUP)
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

.depend: Makefile chip/Make.defs $(SRCS)
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" depend
endif
	$(Q) $(MKDEP) --dep-path chip --dep-path common --dep-path $(ARCH_SUBDIR) \
	 "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" clean
endif
	$(call DELFILE, libarch$(LIBEXT))
	$(call CLEAN)

distclean: clean
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" distclean
endif
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

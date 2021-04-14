############################################################################
# arch/renesas/src/CMakeLists.txt
#
#   Copyright (C) 2008, 2011-2012, 2014, 2016 Gregory Nutt. All rights reserved.
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
############################################################################

-include $(TOPDIR)/Make.defs
-include chip/Make.defs

CPPFLAGS += $(EXTRADEFINES)
CFLAGS += $(EXTRADEFINES)
CXXFLAGS += $(EXTRADEFINES)

if("${CONFIG_WINDOWS_NATIVE}" STREQUAL "y")
  ARCH_SRCDIR = $(TOPDIR)\arch\$(CONFIG_ARCH)\src
  NUTTX = $(TOPDIR)\nuttx$(EXEEXT)
  CFLAGS += -I$(ARCH_SRCDIR)\chip
  CFLAGS += -I$(ARCH_SRCDIR)\common
  CFLAGS += -I$(ARCH_SRCDIR)\$(ARCH_SUBDIR)
  CFLAGS += -I$(TOPDIR)\sched
else()
  ARCH_SRCDIR = $(TOPDIR)/arch/$(CONFIG_ARCH)/src
ifeq ($(WINTOOL}" STREQUAL "y")
  NUTTX = "${shell cygpath -w $(TOPDIR)/nuttx$(EXEEXT)}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/chip}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/common}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/$(ARCH_SUBDIR)}"
  CFLAGS += -I "${shell cygpath -w $(TOPDIR)/sched}"
else()
  NUTTX = $(TOPDIR)/nuttx$(EXEEXT)
  CFLAGS += -I$(ARCH_SRCDIR)/chip
  CFLAGS += -I$(ARCH_SRCDIR)/common
  CFLAGS += -I$(ARCH_SRCDIR)/$(ARCH_SUBDIR)
  CFLAGS += -I$(TOPDIR)/sched
endif()
endif()

CFLAGS += -I$(ARCH_SRCDIR)/chip -I$(ARCH_SRCDIR)/common -I$(TOPDIR)/sched

HEAD_OBJ = $(HEAD_ASRC:.S=$(OBJEXT))
STARTUP_OBJS ?= $(HEAD_OBJ)

ASRCS = $(CHIP_ASRCS) $(CMN_ASRCS)
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = $(CHIP_CSRCS) $(CMN_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

# Override in Make.defs if linker is not 'ld'

LDSTARTGROUP ?= --start-group
LDENDGROUP ?= --end-group

LDFLAGS += $(ARCHSCRIPT)
EXTRA_LIBS ?=
LINKLIBS ?=

if("${CONFIG_WINDOWS_NATIVE}" STREQUAL "y")
  BOARDMAKE = $(if $(wildcard .\board\CMakeLists.txt),y,)
  LIBPATHS += -L"$(TOPDIR)\staging"
ifeq ($(BOARDMAKE}" STREQUAL "y")
  LIBPATHS += -L"$(TOPDIR)\arch\$(CONFIG_ARCH)\src\board"
endif()

else()
  BOARDMAKE = $(if $(wildcard ./board/CMakeLists.txt),y,)

ifeq ($(WINTOOL}" STREQUAL "y")
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)/staging"}"
ifeq ($(BOARDMAKE}" STREQUAL "y")
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)/arch/$(CONFIG_ARCH)/src/board"}"
endif()

else()
  LIBPATHS += -L"$(TOPDIR)/staging"
ifeq ($(BOARDMAKE}" STREQUAL "y")
  LIBPATHS += -L"$(TOPDIR)/arch/$(CONFIG_ARCH)/src/board"
endif()
endif()
endif()

LDLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))
ifeq ($(BOARDMAKE}" STREQUAL "y")
  LDLIBS += -lboard
endif()

LIBGCC = ${shell "$(CC)" $(ARCHCPUFLAGS) -print-libgcc-file-name}

VPATH = chip:common

all: $(HEAD_OBJ) libarch$(LIBEXT)

.PHONY: board/libboard$(LIBEXT)

$(AOBJS) $(HEAD_OBJ): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libarch$(LIBEXT): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

board/libboard$(LIBEXT):
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" libboard$(LIBEXT) EXTRADEFINES=$(EXTRADEFINES)

nuttx$(EXEEXT): $(HEAD_OBJ) board/libboard$(LIBEXT)
	@echo "LD: nuttx"
	$(Q) $(LD) --entry=__start $(LDFLAGS) $(LIBPATHS) -o $(TOPDIR)/$@ $(HEAD_OBJ) \
		$(LDSTARTGROUP) $(LDLIBS) $(EXTRA_LIBS) $(LIBGCC) $(LDENDGROUP)
ifneq ($(CONFIG_WINDOWS_NATIVE}" STREQUAL "y")
	$(Q) $(NM) $(TOPDIR)/$@ | \
	grep -v '\(compiled\)\|\(\$(OBJEXT)$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
	sort > $(TOPDIR)/System.map
endif()

# This is part of the top-level export target

export_startup: board/libboard$(LIBEXT) $(STARTUP_OBJS)
	$(Q) if [ -d "$(EXPORT_DIR)/startup" ]; then \
		cp -f $(STARTUP_OBJS) "$(EXPORT_DIR)/startup"; \
	 else \
		echo "$(EXPORT_DIR)/startup does not exist"; \
		exit 1; \
	 fi

# Dependencies

.depend: CMakeLists.txt chip/Make.defs $(SRCS)
ifeq ($(BOARDMAKE}" STREQUAL "y")
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" depend
endif()
	$(Q) $(MKDEP) --dep-path chip --dep-path common "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
ifeq ($(BOARDMAKE}" STREQUAL "y")
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" clean
endif()
	$(call DELFILE, libarch$(LIBEXT))
	$(call CLEAN)

distclean: clean
ifeq ($(BOARDMAKE}" STREQUAL "y")
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" distclean
endif()
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

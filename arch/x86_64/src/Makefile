############################################################################
# arch/x86_64/src/Makefile
#
# SPDX-License-Identifier: Apache-2.0
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

include $(TOPDIR)/tools/Make.defs
include chip/Make.defs

ifeq ($(CONFIG_ARCH_INTEL64),y)
ARCH_SUBDIR = intel64
endif

ARCH_SRCDIR = $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src

INCLUDES += ${INCDIR_PREFIX}$(ARCH_SRCDIR)$(DELIM)chip
INCLUDES += ${INCDIR_PREFIX}$(ARCH_SRCDIR)$(DELIM)common
INCLUDES += ${INCDIR_PREFIX}$(ARCH_SRCDIR)$(DELIM)$(ARCH_SUBDIR)
INCLUDES += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)sched

CPPFLAGS += $(INCLUDES)
CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)
AFLAGS += $(INCLUDES)

NUTTX = $(call CONVERT_PATH,$(TOPDIR)$(DELIM)nuttx$(EXEEXT))

ASRCS = $(CHIP_ASRCS) $(CMN_ASRCS)
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = $(CHIP_CSRCS) $(CMN_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

KBIN = libkarch$(LIBEXT)
BIN  = libarch$(LIBEXT)

# Override in Make.defs if linker is not 'ld'

$(foreach lib,$(notdir $(wildcard $(APPDIR)$(DELIM)staging$(DELIM)*$(LIBEXT))), \
  $(foreach elib,$(EXTRA_LIBS), \
    $(if $(filter $(notdir $(elib)),$(lib)), \
      $(eval NAMEFULL_LIBS+=$(elib)), \
      $(if $(filter $(notdir $(elib)),$(patsubst lib%$(LIBEXT),-l%,$(lib))), \
        $(eval NAMESPEC_LIBS+=$(elib)) \
       ) \
     ) \
   ) \
 )

EXTRA_LIBS := $(filter-out $(NAMEFULL_LIBS) $(NAMESPEC_LIBS),$(EXTRA_LIBS))
EXTRA_LIBS += $(wildcard $(APPDIR)$(DELIM)staging$(DELIM)*$(LIBEXT))

LDSTARTGROUP ?= --start-group
LDENDGROUP ?= --end-group

LDFLAGS += -z max-page-size=0x1000 $(addprefix -T,$(call CONVERT_PATH,$(ARCHSCRIPT)))
LOADER_LDFLAGS += $(LOADER_ARCHSCRIPT)

BOARDMAKE = $(if $(wildcard board$(DELIM)Makefile),y,)

LIBPATHS += -L $(call CONVERT_PATH,$(TOPDIR)$(DELIM)staging)
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -L $(call CONVERT_PATH,$(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)board)
endif

LDLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))
ifeq ($(BOARDMAKE),y)
  LDLIBS += -lboard
endif

# Add the builtin library

EXTRA_LIBS += ${wildcard ${shell $(CC) $(ARCHCPUFLAGS) --print-libgcc-file-name}}

ifeq ($(CONFIG_LIBM_TOOLCHAIN),y)
  EXTRA_LIBS += ${wildcard ${shell $(CC) $(ARCHCPUFLAGS) --print-file-name=libm.a}}
endif

ifeq ($(CONFIG_LIBSUPCXX_TOOLCHAIN),y)
  EXTRA_LIBS += ${wildcard ${shell $(CC) $(ARCHCPUFLAGS) --print-file-name=libsupc++.a}}
endif

ifeq ($(CONFIG_CXX_EXCEPTION),y)
  EXTRA_LIBS += ${wildcard ${shell $(CC) $(ARCHCPUFLAGS) --print-file-name=libgcc_eh.a}}
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

ifeq ($(CONFIG_ALLSYMS),y)
EXTRA_LIBS += allsyms$(OBJEXT)
endif

ifeq ($(CONFIG_MM_KASAN_GLOBAL),y)
EXTRA_LIBS += kasan_globals$(OBJEXT)
endif

define LINK_ALLSYMS_KASAN
	$(if $(CONFIG_ALLSYMS),
	$(Q) $(TOPDIR)/tools/mkallsyms.py $(NUTTX) allsyms.tmp --orderbyname $(CONFIG_SYMTAB_ORDEREDBYNAME)
	$(Q) $(call COMPILE, allsyms.tmp, allsyms$(OBJEXT), -x c)
	$(Q) $(call DELFILE, allsyms.tmp))
	$(if $(CONFIG_MM_KASAN_GLOBAL),
	$(Q) $(TOPDIR)/tools/kasan_global.py -e $(NUTTX) -o kasan_globals.tmp -a $(CONFIG_MM_KASAN_GLOBAL_ALIGN)
	$(Q) $(call COMPILE, kasan_globals.tmp, kasan_globals$(OBJEXT) -fno-sanitize=kernel-address, -x c)
	$(Q) $(call DELFILE, kasan_globals.tmp))
	$(Q) $(LD) $(LDFLAGS) $(LIBPATHS) $(EXTRA_LIBPATHS) \
		-o $(NUTTX) $(HEAD_OBJ) $(EXTRA_OBJS) \
		$(LDSTARTGROUP) $(EXTRA_LIBS) --no-relax $(LDLIBS) $(LDENDGROUP)
endef

$(addsuffix .tmp,$(ARCHSCRIPT)): $(ARCHSCRIPT)
	$(call PREPROCESS, $(patsubst %.tmp,%,$@), $@)

nuttx$(EXEEXT): $(HEAD_OBJ) board/libboard$(LIBEXT) $(addsuffix .tmp,$(ARCHSCRIPT))
	@echo "LD: nuttx$(EXEEXT)"

ifeq ($(CONFIG_ALLSYMS)$(CONFIG_MM_KASAN_GLOBAL),)
	$(Q) $(LD) $(LDFLAGS) $(LIBPATHS) $(EXTRA_LIBPATHS) \
		-o $(NUTTX) $(HEAD_OBJ) $(EXTRA_OBJS) \
		$(LDSTARTGROUP) $(EXTRA_LIBS) --no-relax $(LDLIBS) $(LDENDGROUP)
else
	$(Q) $(call LINK_ALLSYMS_KASAN)
	$(Q) $(call LINK_ALLSYMS_KASAN)
	$(Q) $(call LINK_ALLSYMS_KASAN)
	$(Q) $(call LINK_ALLSYMS_KASAN)
endif
	$(Q) $(call DELFILE, $(addsuffix .tmp,$(ARCHSCRIPT)))

ifneq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) $(NM) $(NUTTX) | \
	grep -v '\(compiled\)\|\(\$(OBJEXT)$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
	sort > $(TOPDIR)/System.map
endif

ifeq ($(CONFIG_ARCH_MULTIBOOT1),y)
	@echo "Generating: nuttx32 in ELF32/multiboot1"
	$(Q) $(OBJCOPY) -R.realmode -R.note.* -O binary $(NUTTX) $(NUTTX).bin \
	&& $(OBJCOPY) -j.realmode -O binary $(NUTTX) $(NUTTX)_realmode.bin \
	&& $(CC) -m32 -no-pie -nostdlib common/multiboot1.S \
			-T common/multiboot1.ld -o $(NUTTX)32
endif

# This is part of the top-level export target

export_startup: $(STARTUP_OBJS)
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

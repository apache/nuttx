############################################################################
# arch/arm/src/Makefile
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

ifeq ($(CONFIG_ARCH_ARMV7A),y)          # ARMv7-A
ARCH_SUBDIR = armv7-a
else ifeq ($(CONFIG_ARCH_ARMV7R),y)     # ARMv7-R
ARCH_SUBDIR = armv7-r
else ifeq ($(CONFIG_ARCH_ARMV7M),y)     # ARMv7-M
ARCH_SUBDIR = armv7-m
else ifeq ($(CONFIG_ARCH_ARMV8M),y)     # ARMv8-M
ARCH_SUBDIR = armv8-m
else ifeq ($(CONFIG_ARCH_ARMV6M),y)     # ARMv6-M
ARCH_SUBDIR = armv6-m
else                                    # ARM9, ARM7TDMI, etc.
ARCH_SUBDIR = arm
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

# Override in Make.defs if linker is not 'ld'

ifeq ($(CONFIG_ARM_TOOLCHAIN_ARMCLANG),)
  ifeq ($(LD),$(CC))
    LDSTARTGROUP ?= -Wl,--start-group
    LDENDGROUP   ?= -Wl,--end-group
    LDFLAGS      := $(addprefix -Xlinker ,$(LDFLAGS))
    LDFLAGS      += $(CFLAGS)
  else
    LDSTARTGROUP ?= --start-group
    LDENDGROUP   ?= --end-group
  endif

  LIBPATH_OPT = -L
  LIBRARY_OPT = -l
  SCRIPT_OPT  = -T
else
  LIBPATH_OPT = --userlibpath
  LIBRARY_OPT = --library=
  SCRIPT_OPT  = --scatter=
  EXTRA_LIBS += arm_vectors.o
endif

ARCHSCRIPT := $(call CONVERT_PATH,$(ARCHSCRIPT))
LDFLAGS += $(addprefix $(SCRIPT_OPT),$(addsuffix .tmp,$(ARCHSCRIPT))) $(EXTRALINKCMDS)
LIBPATHS += $(LIBPATH_OPT) $(call CONVERT_PATH,$(TOPDIR)$(DELIM)staging)

BOARDMAKE = $(if $(wildcard board$(DELIM)Makefile),y,)
ifeq ($(BOARDMAKE),y)
  LIBPATHS += $(LIBPATH_OPT) $(call CONVERT_PATH,$(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)board)
endif

LDLIBS = $(patsubst %.a,%,$(patsubst lib%,$(LIBRARY_OPT)%,$(LINKLIBS)))
ifeq ($(BOARDMAKE),y)
  LDLIBS += $(LIBRARY_OPT)board
endif

VPATH += chip
VPATH += common
VPATH += $(ARCH_SUBDIR)

ifeq ($(CONFIG_ARM_TOOLCHAIN_IAR),y)
  VPATH += common$(DELIM)iar
else # ifeq ($(CONFIG_ARCH_TOOLCHAIN_GNU),y)
  VPATH += common$(DELIM)gnu
endif

all: $(HEAD_OBJ) $(BIN)

.PHONY: board$(DELIM)libboard$(LIBEXT)

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

board$(DELIM)libboard$(LIBEXT):
	$(Q) $(MAKE) -C board libboard$(LIBEXT) EXTRAFLAGS="$(EXTRAFLAGS)"

define LINK_ALLSYMS
	$(Q) $(TOPDIR)/tools/mkallsyms.py $(NUTTX) allsyms.tmp
	$(Q) $(call COMPILE, -x c allsyms.tmp, allsyms$(OBJEXT))
	$(Q) $(LD) --entry=__start $(LDFLAGS) $(LIBPATHS) $(EXTRA_LIBPATHS) \
		-o $(NUTTX) $(HEAD_OBJ) allsyms$(OBJEXT) $(EXTRA_OBJS) \
		$(LDSTARTGROUP) $(LDLIBS) $(EXTRA_LIBS) $(LDENDGROUP)
	$(Q) $(call DELFILE, allsyms.tmp allsyms$(OBJEXT))
endef

$(addsuffix .tmp,$(ARCHSCRIPT)): $(ARCHSCRIPT)
	$(call PREPROCESS, $(patsubst %.tmp,%,$@), $@)

nuttx$(EXEEXT): $(HEAD_OBJ) board$(DELIM)libboard$(LIBEXT) $(addsuffix .tmp,$(ARCHSCRIPT))
	$(Q) echo "LD: nuttx"
ifneq ($(CONFIG_ALLSYMS),y)
	$(Q) $(LD) --entry=__start $(LDFLAGS) $(LIBPATHS) $(EXTRA_LIBPATHS) \
		-o $(NUTTX) $(HEAD_OBJ) $(EXTRA_OBJS) \
		$(LDSTARTGROUP) $(LDLIBS) $(EXTRA_LIBS) $(LDENDGROUP)
else
	$(Q) # Link and generate default table
	$(Q) $(if $(wildcard $(shell echo $(NUTTX))),,$(call LINK_ALLSYMS,$^))
	$(Q) # Extract all symbols
	$(Q) $(call LINK_ALLSYMS, $^)
	$(Q) # Extract again since the table offset may changed
	$(Q) $(call LINK_ALLSYMS, $^)
endif
ifneq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) $(NM) $(NUTTX) | \
	grep -v '\(compiled\)\|\(\$(OBJEXT)$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
	sort > $(TOPDIR)$(DELIM)System.map
endif
	$(Q) $(call DELFILE, $(addsuffix .tmp,$(ARCHSCRIPT)))

# This is part of the top-level export target
# Note that there may not be a head object if layout is handled
# by the linker configuration.

export_startup: $(STARTUP_OBJS)
ifneq ($(STARTUP_OBJS),)
	$(Q) if [ -d "$(EXPORT_DIR)$(DELIM)startup" ]; then \
		cp -f $(STARTUP_OBJS) "$(EXPORT_DIR)$(DELIM)startup$(DELIM)."; \
	 else \
		echo "$(EXPORT_DIR)$(DELIM)startup does not exist"; \
	exit 1; \
	fi
endif

# Dependencies

makedepfile: $(CSRCS:.c=.ddc) $(ASRCS:.S=.dds) $(HEAD_ASRC:.S=.dds)
	$(call CATFILE, Make.dep, $^)
	$(call DELFILE, $^)

.depend: Makefile chip$(DELIM)Make.defs $(SRCS) $(TOPDIR)$(DELIM).config
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
	$(call DELFILE, $(addsuffix .tmp,$(ARCHSCRIPT)))
	$(call DELFILE, $(KBIN))
	$(call DELFILE, $(BIN))
ifneq ($(EXTRADELFILE),)
	$(call DELFILE, $(EXTRADELFILE))
endif
	$(call CLEAN)

distclean:: clean
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board distclean
endif
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

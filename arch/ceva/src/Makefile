############################################################################
# arch/ceva/src/Makefile
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
-include chip$(DELIM)Make.defs

ifeq ($(CONFIG_ARCH_XC5),y)
ARCH_SUBDIR = xc5
else ifeq ($(CONFIG_ARCH_XM6),y)
ARCH_SUBDIR = xm6
endif

CPPFLAGS += $(EXTRADEFINES)
CFLAGS += $(EXTRADEFINES)
CXXFLAGS += $(EXTRADEFINES)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  ARCH_SRCDIR = $(TOPDIR)\arch\$(CONFIG_ARCH)\src
  NUTTX = "$(TOPDIR)\nuttx$(EXEEXT)"
  CFLAGS += -I$(ARCH_SRCDIR)\chip
  CFLAGS += -I$(ARCH_SRCDIR)\common
  CFLAGS += -I$(ARCH_SRCDIR)\$(ARCH_SUBDIR)
  CFLAGS += -I$(ARCH_SRCDIR)\$(CONFIG_ARCH_CHIP)
  CFLAGS += -I$(TOPDIR)\sched
else
  ARCH_SRCDIR = $(TOPDIR)/arch/$(CONFIG_ARCH)/src
  NUTTX = $(call CONVERT_PATH,$(TOPDIR)/nuttx$(EXEEXT))
  CFLAGS += -I $(call CONVERT_PATH,$(ARCH_SRCDIR)/chip)
  CFLAGS += -I $(call CONVERT_PATH,$(ARCH_SRCDIR)/common)
  CFLAGS += -I $(call CONVERT_PATH,$(ARCH_SRCDIR)/$(ARCH_SUBDIR))
  CFLAGS += -I $(call CONVERT_PATH,$(ARCH_SRCDIR)/$(CONFIG_ARCH_CHIP))
  CFLAGS += -I $(call CONVERT_PATH,$(TOPDIR)/sched)
  LDSCRIPT := $(call CONVERT_PATH,$(LDSCRIPT))
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
UBIN = libuarch$(LIBEXT)
BIN  = libarch$(LIBEXT)

EXTRA_LIBS ?=
EXTRA_LIBPATHS ?=
LINKLIBS ?=

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BOARDMAKE = $(if $(wildcard .\board\Makefile),y,)
  LIBPATHS += -I "$(TOPDIR)\staging"
  ifeq ($(BOARDMAKE),y)
    LIBPATHS += -I "$(TOPDIR)\arch\$(CONFIG_ARCH)\src\board"
  endif
else
  BOARDMAKE = $(if $(wildcard ./board/Makefile),y,)
  LIBPATHS += -I $(call CONVERT_PATH,$(TOPDIR)/staging)
  ifeq ($(BOARDMAKE),y)
    LIBPATHS += -I $(call CONVERT_PATH,$(TOPDIR)/arch/$(CONFIG_ARCH)/src/board)
  endif
endif

LDLIBS = $(patsubst %, -lib %,$(LINKLIBS))
ifeq ($(BOARDMAKE),y)
  LDLIBS += -lib libboard.lib
endif

VPATH += .
VPATH += chip
VPATH += common
VPATH += $(ARCH_SUBDIR)

VPATH := $(patsubst %,:$(SRCDIR)$(DELIM)%,$(VPATH))
VPATH += :$(call CONVERT_PATH,$(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)chip)

all: $(HEAD_OBJ) $(BIN)

.PHONY: board$(DELIM)libboard$(LIBEXT)

$(sort $(AOBJS) $(UAOBJS) $(HEAD_OBJ)): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(sort $(COBJS) $(UCOBJS)): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN) $(KBIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

$(UBIN): $(UOBJS)
	$(call ARCHIVE, $@, $(UOBJS))

board$(DELIM)libboard$(LIBEXT):
ifeq ($(CONFIG_ARCH_XC5), y)
	$(Q) if [ ! -L "$(CROSSDEV)/cevaxccc-xdrv" ]; then \
		ln -sf "$(CROSSDEV)/cevaxccc-drv"  "$(CROSSDEV)/cevaxccc-xdrv";\
	fi
endif
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" libboard$(LIBEXT) EXTRADEFINES=$(EXTRADEFINES)

nuttx$(EXEEXT): $(HEAD_OBJ) board$(DELIM)libboard$(LIBEXT) $(LDSCRIPT)
	$(Q) echo "LD: nuttx"
	$(Q) $(call PREPROCESS, $(LDSCRIPT), $(notdir $(LDSCRIPT)))
	$(Q) $(LD) $(LDFLAGS) $(LIBPATHS) $(EXTRA_LIBPATHS) \
		-o $(NUTTX) $(HEAD_OBJ) $(EXTRA_OBJS) \
		$(LDLIBS) $(EXTRA_LIBS) $(LIBGCC) \
		-l $(TOPDIR)$(DELIM)nuttx.lin $(notdir $(LDSCRIPT))
	$(Q) $(DISASM) -o $(TOPDIR)$(DELIM)nuttx.lst $(NUTTX)
	$(Q) $(OBJDUMP) $(NUTTX) > $(TOPDIR)$(DELIM)nuttx.dump
	$(Q) $(OBJCOPY) -b $(TOPDIR)$(DELIM)nuttx -c -split $(NUTTX)

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

.depend: Makefile chip$(DELIM)Make.defs $(SRCS)
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" depend
endif
	$(Q) $(MKDEP) $(patsubst %,--dep-path %,$(subst :, ,$(VPATH))) \
	 "$(CC)" -- $(CFLAGS) -- $^ >Make.dep
	$(Q) touch $@

depend: .depend

clean:
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" clean
endif
	$(call DELFILE, $(notdir $(LDSCRIPT)))
	$(call DELFILE, $(KBIN))
	$(call DELFILE, $(UBIN))
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" distclean
endif
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

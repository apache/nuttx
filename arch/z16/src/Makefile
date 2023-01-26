############################################################################
# arch/z16/src/Makefile
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

COMPILER = ${shell basename "$(CC)"}
ARCHSRCDIR = $(TOPDIR)/arch/$(CONFIG_ARCH)/src

INCLUDES += ${INCDIR_PREFIX}$(ARCHSRCDIR)$(DELIM)chip
INCLUDES += ${INCDIR_PREFIX}$(ARCHSRCDIR)$(DELIM)common
INCLUDES += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)sched

CFLAGS += $(INCLUDES)
CPPFLAGS += $(INCLUDES)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  LDFLAGS += @"$(ARCHSRCDIR)/nuttx.linkcmd"
else
ifeq ($(COMPILER),zneocc.exe)
  LDFLAGS += @"${shell cygpath -w $(ARCHSRCDIR)/nuttx.linkcmd}"
endif
endif

HEAD_ASRC = $(HEAD_SSRC:.S=$(ASMEXT))
HEAD_OBJ = $(HEAD_SSRC:.S=$(OBJEXT))
STARTUP_OBJS ?= $(HEAD_OBJ)

SSRCS = $(CHIP_SSRCS) $(CMN_SSRCS)
ASRCS = $(SSRCS:.S=$(ASMEXT))
AOBJS = $(SSRCS:.S=$(OBJEXT))

CSRCS = $(CHIP_CSRCS) $(CMN_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

DEPSRCS = $(SSRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

VPATH = chip:common

all: $(HEAD_OBJ) libarch$(LIBEXT)

.PHONY: board/libboard$(LIBEXT)

ifeq ($(COMPILER),zneocc.exe)
$(ASRCS) $(HEAD_ASRC): %$(ASMEXT): %.S
	$(Q) $(CPP) $(CPPFLAGS) $< -o $@.tmp
	$(Q) cat $@.tmp | sed -e "s/^#/;/g" > $@
	$(Q) rm $@.tmp

$(AOBJS) $(HEAD_OBJ): %$(OBJEXT): %$(ASMEXT)
	$(call ASSEMBLE, $<, $@)
else
$(OBJS) $(HEAD_OBJ): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)
endif

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libarch$(LIBEXT): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

board/libboard$(LIBEXT):
	$(Q) $(MAKE) -C board libboard$(LIBEXT) EXTRAFLAGS="$(EXTRAFLAGS)"

ifeq ($(COMPILER),zneocc.exe)
nuttx.linkcmd: $(LINKCMDTEMPLATE)
	$(Q) cp -f $(LINKCMDTEMPLATE) nuttx.linkcmd
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
	@echo "$(TOPDIR)\nuttx"= \>>nuttx.linkcmd
	@echo   "$(ARCHSRCDIR)\$(HEAD_OBJ)", \>>nuttx.linkcmd
	$(Q) for %%G in ($(LINKLIBS)) do ( echo   "$(TOPDIR)\staging\%%G", \>>nuttx.linkcmd )
	@echo   "$(ARCHSRCDIR)\board\libboard$(LIBEXT)", \>>nuttx.linkcmd
	@echo   "$(ZDSSTDLIBDIR)\chelpld$(LIBEXT)", \>>nuttx.linkcmd
	@echo   "$(ZDSSTDLIBDIR)\fpld$(LIBEXT)">>nuttx.linkcmd
else
	@echo "\"${shell cygpath -w $(TOPDIR)/nuttx}\"= \\" >>nuttx.linkcmd
	@echo "  \"${shell cygpath -w $(ARCHSRCDIR)/$(HEAD_OBJ)}\", \\" >>nuttx.linkcmd
	$(Q) ( for lib in $(LINKLIBS); do \
		echo "  \"`cygpath -w $(TOPDIR)/staging/$${lib}`\", \\" >>nuttx.linkcmd; \
	done ; )
	@echo "  \"${shell cygpath -w $(ARCHSRCDIR)/board/libboard$(LIBEXT)}\", \\"  >>nuttx.linkcmd
	@echo "  \"${shell cygpath -w $(ZDSSTDLIBDIR)/chelpld$(LIBEXT)}\", \\"  >>nuttx.linkcmd
	@echo "  \"${shell cygpath -w $(ZDSSTDLIBDIR)/fpld$(LIBEXT)}\""  >>nuttx.linkcmd
endif
else
nuttx.linkcmd:
endif

nuttx$(EXEEXT): $(HEAD_OBJ) board/libboard$(LIBEXT) nuttx.linkcmd
	@echo "LD:  nuttx$(EXEEXT)"
	$(Q) $(LD) $(LDFLAGS)

.depend: Makefile chip/Make.defs $(DEPSRCS) $(TOPDIR)$(DELIM).config
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) if exist board$(DELIM)Makefile ( $(MAKE) -C board depend )
else
	$(Q) if [ -e board/Makefile ]; then \
		$(MAKE) -C board depend ; \
	fi
endif
	$(Q) $(MKDEP) --dep-path chip --dep-path common "$(CC)" -- $(CFLAGS) -- $(DEPSRCS) >Make.dep
	$(Q) touch $@

# This is part of the top-level export target

export_startup: $(STARTUP_OBJS)
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) if exist "$(EXPORT_DIR)$(DELIM)startup" ( copy $(STARTUP_OBJS) "$(EXPORT_DIR)$(DELIM)startup$(DELIM)." /b /y)
else
	$(Q) if [ -d "$(EXPORT_DIR)/startup" ]; then \
		cp -f $(STARTUP_OBJS) "$(EXPORT_DIR)/startup"; \
	 else \
		echo "$(EXPORT_DIR)/startup does not exist"; \
		exit 1; \
	 fi
endif

# Dependencies

depend: .depend

context::

clean:
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) if exist board$(DELIM)Makefile ( $(MAKE) -C board clean )
else
	$(Q) if [ -e board/Makefile ]; then \
		$(MAKE) -C board clean ; \
	fi
endif
ifeq ($(COMPILER),zneocc.exe)
	$(call DELFILE, nuttx.linkcmd)
	$(call DELFILE, *.asm)
	$(call DELFILE, *.tmp)
	$(call DELFILE, *.map)
endif
	$(call DELFILE, libarch$(LIBEXT))
	$(call CLEAN)

distclean:: clean
ifeq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) if exist board$(DELIM)Makefile ( $(MAKE) -C board distclean )
else
	$(Q) if [ -e board/Makefile ]; then \
		$(MAKE) -C board distclean ; \
	fi
endif
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep

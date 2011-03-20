############################################################################
# Makefile
#
#   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

TOPDIR		:= ${shell pwd | sed -e 's/ /\\ /g'}
-include ${TOPDIR}/.config
-include ${TOPDIR}/Make.defs

# Default tools

ifeq ($(DIRLINK),)
DIRLINK		= $(TOPDIR)/tools/link.sh
DIRUNLINK	= $(TOPDIR)/tools/unlink.sh
endif

# This is the final executable

ifeq ($(WINTOOL),y)
  NUTTX		= "${shell cygpath -w $(TOPDIR)/nuttx}"
else
  NUTTX		= $(TOPDIR)/nuttx
endif

# Process architecture and board-specific directories

ARCH_DIR	= arch/$(CONFIG_ARCH)
ARCH_SRC	= $(ARCH_DIR)/src
ARCH_INC	= $(ARCH_DIR)/include
BOARD_DIR	= configs/$(CONFIG_ARCH_BOARD)

# Add-on directories.  These may or may not be in place in the
# NuttX source tree (they must be specifically installed)
#
# APPLOC can be over-ridden from the command line or in the .config file:

ifeq ($(CONFIG_BUILTIN_APPS),y)
ifeq ($(APPLOC),)
APPLOC	= ../apps
endif
APPDIR		:= ${shell if [ -r $(APPLOC)/Makefile ]; then echo "$(APPLOC)"; fi}
endif

PCODE_DIR	:= ${shell if [ -r pcode/Makefile ]; then echo "pcode"; fi}
ADDON_DIRS	:= $(PCODE_DIR) $(NX_DIR) $(APPDIR)

# FSDIRS depend on file descriptor support; NONFSDIRS do not
#   (except for parts of FSDIRS).  We will exclude FSDIRS
#   from the build if file descriptor support is disabled

NONFSDIRS	= sched lib $(ARCH_SRC) mm $(ADDON_DIRS)
FSDIRS		= fs drivers binfmt
NETFSDIRS	= fs drivers
CONTEXTDIRS	=

ifeq ($(CONFIG_NX),y)
NONFSDIRS	+= graphics
CONTEXTDIRS	+= graphics
endif

ifneq ($(CONFIG_APP_DIR),)
NONFSDIRS	= $(CONFIG_APP_DIR)
endif

# CLEANDIRS are the directories that will clean in.  These are
#   all directories that we know about.
# MAKEDIRS are the directories in which we will build targets

CLEANDIRS	= $(NONFSDIRS) $(FSDIRS)
MAKEDIRS	= $(NONFSDIRS)

# Add file system directories to MAKEDIRS (they are already in CLEANDIRS)

ifeq ($(CONFIG_NFILE_DESCRIPTORS),0)
ifeq ($(CONFIG_NET),y)
ifneq ($(CONFIG_NSOCKET_DESCRIPTORS),0)
MAKEDIRS	+= fs
endif
MAKEDIRS	+= drivers
endif
else
MAKEDIRS	+= $(FSDIRS)
endif

# Add networking directories to MAKEDIRS and CLEANDIRS

ifeq ($(CONFIG_NET),y)
MAKEDIRS	+= net
endif
CLEANDIRS	+= net

#
# Extra objects used in the final link.
#
# Pass 1 1ncremental (relative) link objects should be put into the
# processor-specific source directory (where other link objects will
# be created).  If the pass1 obect is an archive, it could go anywhere.

ifeq ($(CONFIG_BUILD_2PASS),y)
#EXTRA_OBJS	= $(TOPDIR)/$(CONFIG_PASS1_BUILDIR)/$(CONFIG_PASS1_OBJECT)
EXTRA_OBJS	+= $(CONFIG_PASS1_OBJECT)
endif

# LINKLIBS is the list of NuttX libraries that is passed to the
#   processor-specific Makefile to build the final target.
#   Libraries in FSDIRS are excluded if file descriptor support
#   is disabled.

LINKLIBS	= sched/libsched$(LIBEXT) $(ARCH_SRC)/libarch$(LIBEXT) mm/libmm$(LIBEXT) \
		  lib/liblib$(LIBEXT)

# Add libraries for network support.  CXX, CXXFLAGS, and COMPILEXX must
# be defined in Make.defs for this to work!

ifeq ($(CONFIG_HAVE_CXX),y)
LINKLIBS	+= libxx/liblibxx$(LIBEXT)
endif

# Add application-specific library

ifneq ($(CONFIG_APP_DIR),)
LINKLIBS	= $(CONFIG_APP_DIR)/libapp$(LIBEXT)
endif

# Add library for application support
# Always compile the framework which includes exec_nuttapp if users
# or nuttX applications are to be included.

ifeq ($(CONFIG_BUILTIN_APPS),y)
LINKLIBS	+= $(APPDIR)/libapps$(LIBEXT)
endif

# Add libraries for network support

ifeq ($(CONFIG_NET),y)
LINKLIBS	+= net/libnet$(LIBEXT)
endif

# Add libraries for file system support

ifeq ($(CONFIG_NFILE_DESCRIPTORS),0)
ifneq ($(CONFIG_NSOCKET_DESCRIPTORS),0)
LINKLIBS	+= fs/libfs$(LIBEXT)
endif
ifeq ($(CONFIG_NET),y)
LINKLIBS	+= drivers/libdrivers$(LIBEXT)
endif
else
LINKLIBS	+= fs/libfs$(LIBEXT) drivers/libdrivers$(LIBEXT) binfmt/libbinfmt$(LIBEXT)
endif

# Add libraries for Pascall P-Code

ifneq ($(PCODE_DIR),)
LINKLIBS	+= $(PCODE_DIR)/libpcode$(LIBEXT)
endif

# Add libraries for the NX graphics sub-system

ifneq ($(NX_DIR),)
LINKLIBS	+= $(NX_DIR)/libnx$(LIBEXT)
endif

ifeq ($(CONFIG_NX),y)
LINKLIBS        += graphics/libgraphics$(LIBEXT)
endif

# This is the name of the final target
BIN		= nuttx$(EXEEXT)

all: $(BIN)
.PHONY: context clean_context check_context subdir_clean clean subdir_distclean distclean

# Build the mkconfig tool used to create include/nuttx/config.h
tools/mkconfig:
	@$(MAKE) -C tools -f Makefile.mkconfig TOPDIR="$(TOPDIR)"  mkconfig

# Create the include/nuttx/config.h file
include/nuttx/config.h: $(TOPDIR)/.config tools/mkconfig
	tools/mkconfig $(TOPDIR) > include/nuttx/config.h

# link the arch/<arch-name>/include dir to include/arch
include/arch: Make.defs
	@$(DIRLINK) $(TOPDIR)/$(ARCH_DIR)/include include/arch

# Link the configs/<board-name>/include dir to include/arch/board
include/arch/board: include/arch Make.defs include/arch
	@$(DIRLINK) $(TOPDIR)/$(BOARD_DIR)/include include/arch/board

# Link the configs/<board-name>/src dir to arch/<arch-name>/src/board
$(ARCH_SRC)/board: Make.defs
	@$(DIRLINK) $(TOPDIR)/$(BOARD_DIR)/src $(ARCH_SRC)/board

# Link arch/<arch-name>/include/<chip-name> to arch/<arch-name>/include/chip
$(ARCH_SRC)/chip: Make.defs
ifneq ($(CONFIG_ARCH_CHIP),)
	@$(DIRLINK) $(TOPDIR)/$(ARCH_SRC)/$(CONFIG_ARCH_CHIP) $(ARCH_SRC)/chip
endif

# Link arch/<arch-name>/src/<chip-name> to arch/<arch-name>/src/chip
include/arch/chip: include/arch Make.defs
ifneq ($(CONFIG_ARCH_CHIP),)
	@$(DIRLINK) $(TOPDIR)/$(ARCH_INC)/$(CONFIG_ARCH_CHIP) include/arch/chip
endif

dirlinks: include/arch include/arch/board include/arch/chip $(ARCH_SRC)/board $(ARCH_SRC)/chip

context: check_context include/nuttx/config.h dirlinks
	@for dir in $(CONTEXTDIRS) ; do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" context ; \
	done

clean_context:
	@rm -f include/nuttx/config.h
	@$(DIRUNLINK) include/arch/board
	@$(DIRUNLINK) include/arch/chip
	@$(DIRUNLINK) include/arch
	@$(DIRUNLINK) $(ARCH_SRC)/board
	@$(DIRUNLINK) $(ARCH_SRC)/chip

check_context:
	@if [ ! -e ${TOPDIR}/.config -o ! -e ${TOPDIR}/Make.defs ]; then \
		echo "" ; echo "Nuttx has not been configured:" ; \
		echo "  cd tools; ./configure.sh <target>\n" ; echo "" ;\
		exit 1 ; \
	fi

sched/libsched$(LIBEXT): context
	@$(MAKE) -C sched TOPDIR="$(TOPDIR)" libsched$(LIBEXT)

lib/liblib$(LIBEXT): context
	@$(MAKE) -C lib TOPDIR="$(TOPDIR)" liblib$(LIBEXT)

libxx/liblibxx$(LIBEXT): context
	@$(MAKE) -C libxx TOPDIR="$(TOPDIR)" liblibxx$(LIBEXT)

$(ARCH_SRC)/libarch$(LIBEXT): context
	@$(MAKE) -C $(ARCH_SRC) TOPDIR="$(TOPDIR)" libarch$(LIBEXT)

mm/libmm$(LIBEXT): context
	@$(MAKE) -C mm TOPDIR="$(TOPDIR)" libmm$(LIBEXT)

net/libnet$(LIBEXT): context
	@$(MAKE) -C net TOPDIR="$(TOPDIR)" libnet$(LIBEXT)

fs/libfs$(LIBEXT): context
	@$(MAKE) -C fs TOPDIR="$(TOPDIR)" libfs$(LIBEXT)

drivers/libdrivers$(LIBEXT): context
	@$(MAKE) -C drivers TOPDIR="$(TOPDIR)" libdrivers$(LIBEXT)

$(APPDIR)/libapps$(LIBEXT): context
	@$(MAKE) -C $(APPDIR) TOPDIR="$(TOPDIR)" libapps$(LIBEXT)

binfmt/libbinfmt$(LIBEXT): context
	@$(MAKE) -C binfmt TOPDIR="$(TOPDIR)" libbinfmt$(LIBEXT)

pcode/libpcode$(LIBEXT): context
	@$(MAKE) -C pcode TOPDIR="$(TOPDIR)" libpcode$(LIBEXT)

graphics/libgraphics$(LIBEXT): context
	@$(MAKE) -C graphics TOPDIR="$(TOPDIR)" libgraphics$(LIBEXT)

$(CONFIG_APP_DIR)/libapp$(LIBEXT): context
	@$(MAKE) -C $(CONFIG_APP_DIR) TOPDIR="$(TOPDIR)" libapp$(LIBEXT)

# If the 2 pass build option is selected, then this pass1 target is
# configured be build a extra link object. This is assumed to be an
# incremental (relative) link object, but could be a static library
# (archive); some modification to this Makefile would be required if
# CONFIG_PASS1_OBJECT is an archive.

pass1:
ifeq ($(CONFIG_BUILD_2PASS),y)
	@if [ -z "$(CONFIG_PASS1_OBJECT)" ]; then \
		echo "ERROR: CONFIG_PASS1_OBJECT not defined"; \
		exit 1; \
	fi
	@if [ -z "$(CONFIG_PASS1_BUILDIR)" ]; then \
		echo "ERROR: CONFIG_PASS1_BUILDIR not defined"; \
		exit 1; \
	fi
	@if [ ! -d "$(CONFIG_PASS1_BUILDIR)" ]; then \
		echo "ERROR: CONFIG_PASS1_BUILDIR does not exist"; \
		exit 1; \
	fi
	@if [ ! -f "$(CONFIG_PASS1_BUILDIR)/Makefile" ]; then \
		echo "ERROR: No Makefile in CONFIG_PASS1_BUILDIR"; \
		exit 1; \
	fi
	@$(MAKE) -C $(CONFIG_PASS1_BUILDIR) TOPDIR="$(TOPDIR)" LINKLIBS="$(LINKLIBS)" "$(ARCH_SRC)/$(CONFIG_PASS1_OBJECT)"
endif

$(BIN):	context depend $(LINKLIBS) pass1
	@$(MAKE) -C $(ARCH_SRC) TOPDIR="$(TOPDIR)" EXTRA_OBJS="$(EXTRA_OBJS)" LINKLIBS="$(LINKLIBS)" $(BIN)
	@if [ -w /tftpboot ] ; then \
		cp -f $(TOPDIR)/$@ /tftpboot/$@.${CONFIG_ARCH}; \
	fi
ifeq ($(CONFIG_RRLOAD_BINARY),y)
	@$(TOPDIR)/tools/mkimage.sh --Prefix $(CROSSDEV) $(TOPDIR)/$@ $(TOPDIR)/$@.rr
	@if [ -w /tftpboot ] ; then \
		cp -f $(TOPDIR)/$@.rr /tftpboot/$@.rr.${CONFIG_ARCH}; \
	fi
endif
ifeq ($(CONFIG_INTELHEX_BINARY),y)
	@$(OBJCOPY) $(OBJCOPYARGS) -O ihex $(NUTTX)$(EXEEXT) $(NUTTX)$(EXEEXT).ihx
endif
ifeq ($(CONFIG_MOTOROLA_SREC),y)
	@$(OBJCOPY) $(OBJCOPYARGS) -O srec $(NUTTX)$(EXEEXT) $(NUTTX)$(EXEEXT).srec
endif
ifeq ($(CONFIG_RAW_BINARY),y)
	@$(OBJCOPY) $(OBJCOPYARGS) -O binary $(NUTTX)$(EXEEXT) $(NUTTX)$(EXEEXT).bin
endif

# This is a helper target that will rebuild NuttX and download it to the
# target system in one step.  It will generate an error an error if the
# DOWNLOAD command is not defined in platform Make.defs file.

download: $(BIN)
	$(call DOWNLOAD, $<)

depend: context
	@for dir in $(MAKEDIRS) ; do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" depend ; \
	done

subdir_clean:
	@for dir in $(CLEANDIRS) ; do \
		if [ -e $$dir/Makefile ]; then \
			$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" clean ; \
		fi \
	done
	@$(MAKE) -C tools -f Makefile.mkconfig TOPDIR="$(TOPDIR)" clean
	@$(MAKE) -C mm -f Makefile.test TOPDIR="$(TOPDIR)" clean
ifeq ($(CONFIG_BUILD_2PASS),y)
	@$(MAKE) -C $(CONFIG_PASS1_BUILDIR) TOPDIR="$(TOPDIR)" clean
endif

clean: subdir_clean
	@rm -f $(BIN) nuttx.* mm_test *.map *~

subdir_distclean:
	@for dir in $(CLEANDIRS) ; do \
		if [ -e $$dir/Makefile ]; then \
			$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" distclean ; \
		fi \
	done

distclean: clean subdir_distclean clean_context
ifeq ($(CONFIG_BUILD_2PASS),y)
	@$(MAKE) -C $(CONFIG_PASS1_BUILDIR) TOPDIR="$(TOPDIR)" distclean
endif
	@rm -f Make.defs setenv.sh .config


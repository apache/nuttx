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
# CONFIG_APPS_DIR can be over-ridden from the command line or in the .config file.
# The default value of CONFIG_APPS_DIR is ../apps.  Ultimately, the application
# will be built if APPDIR is defined.  APPDIR will be defined if a directory containing
# a Makefile is found at the path provided by CONFIG_APPS_DIR

ifeq ($(CONFIG_APPS_DIR),)
CONFIG_APPS_DIR	= ../apps
endif
APPDIR		:= ${shell if [ -r $(CONFIG_APPS_DIR)/Makefile ]; then echo "$(CONFIG_APPS_DIR)"; fi}

# The Pascal p-code add-on directory

PCODE_DIR	:= ${shell if [ -r pcode/Makefile ]; then echo "pcode"; fi}

# All add-on directories.
#
# NUTTX_ADDONS is the list of directories built into the NuttX kernel.
# USER_ADDONS is the list of directories that will be built into the user application

NUTTX_ADDONS	:= $(PCODE_DIR) $(NX_DIR)
USER_ADDONS		:=

ifeq ($(CONFIG_NUTTX_KERNEL),y)
USER_ADDONS		+= $(APPDIR)
else
NUTTX_ADDONS	+= $(APPDIR)
endif

# Lists of build directories.
#
# FSDIRS depend on file descriptor support; NONFSDIRS do not (except for parts
#   of FSDIRS).  We will exclude FSDIRS from the build if file descriptor
#   support is disabled
# CONTEXTDIRS include directories that have special, one-time pre-build
#   requirements.  Normally this includes things like auto-generation of
#   configuration specific files or creation of configurable symbolic links
# USERDIRS - When NuttX is build is a monolithic kernel, this provides the
#   list of directories that must be built

NONFSDIRS	= sched $(ARCH_SRC) mm $(NUTTX_ADDONS)
FSDIRS		= fs drivers binfmt
NETFSDIRS	= fs drivers
CONTEXTDIRS	= $(APPDIR)
USERDIRS	=

ifeq ($(CONFIG_NUTTX_KERNEL),y)
NONFSDIRS	+= syscall
CONTEXTDIRS	+= syscall
USERDIRS	+= syscall lib $(USER_ADDONS)
else
NONFSDIRS	+= lib
endif

ifeq ($(CONFIG_NX),y)
NONFSDIRS	+= graphics
CONTEXTDIRS	+= graphics
endif

# CLEANDIRS are the directories that will clean in.  These are
#   all directories that we know about.
# KERNDEPDIRS are the directories in which we will build target dependencies.
#   If NuttX and applications are built separately (CONFIG_NUTTX_KERNEL),
#   then this holds only the directories containing kernel files.
# USERDEPDIRS. If NuttX and applications are built separately (CONFIG_NUTTX_KERNEL),
#   then this holds only the directories containing user files.

CLEANDIRS	= $(NONFSDIRS) $(FSDIRS) $(USERDIRS)
KERNDEPDIRS	= $(NONFSDIRS)
USERDEPDIRS	= $(USERDIRS)

# Add file system directories to KERNDEPDIRS (they are already in CLEANDIRS)

ifeq ($(CONFIG_NFILE_DESCRIPTORS),0)
ifeq ($(CONFIG_NET),y)
ifneq ($(CONFIG_NSOCKET_DESCRIPTORS),0)
KERNDEPDIRS	+= fs
endif
KERNDEPDIRS	+= drivers
endif
else
KERNDEPDIRS	+= $(FSDIRS)
endif

# Add networking directories to KERNDEPDIRS and CLEANDIRS

ifeq ($(CONFIG_NET),y)
KERNDEPDIRS	+= net
endif
CLEANDIRS	+= net

#
# Extra objects used in the final link.
#
# Pass 1 1ncremental (relative) link objects should be put into the
# processor-specific source directory (where other link objects will
# be created).  If the pass1 obect is an archive, it could go anywhere.

ifeq ($(CONFIG_BUILD_2PASS),y)
EXTRA_OBJS	+= $(CONFIG_PASS1_OBJECT)
endif

# NUTTXLIBS is the list of NuttX libraries that is passed to the
#   processor-specific Makefile to build the final NuttX target.
#   Libraries in FSDIRS are excluded if file descriptor support
#   is disabled.
# USERLIBS is the list of libraries used to build the final user-space
#   application

NUTTXLIBS	= sched/libsched$(LIBEXT) $(ARCH_SRC)/libarch$(LIBEXT) mm/libmm$(LIBEXT) \
		  lib/liblib$(LIBEXT)
USERLIBS	=

# Add libraries for syscall support.  The C library will be needed by
# both the kernel- and user-space builds.

ifeq ($(CONFIG_NUTTX_KERNEL),y)
NUTTXLIBS	+= syscall/libstubs$(LIBEXT)
USERLIBS	+= syscall/libproxies$(LIBEXT) lib/liblib$(LIBEXT)
endif

# Add libraries for network support.  CXX, CXXFLAGS, and COMPILEXX must
# be defined in Make.defs for this to work!

ifeq ($(CONFIG_HAVE_CXX),y)
ifeq ($(CONFIG_NUTTX_KERNEL),y)
USERLIBS	+= libxx/liblibxx$(LIBEXT)
else
NUTTXLIBS	+= libxx/liblibxx$(LIBEXT)
endif
endif

# Add library for application support.

ifneq ($(APPDIR),)
ifeq ($(CONFIG_NUTTX_KERNEL),y)
USERLIBS	+= $(APPDIR)/libapps$(LIBEXT)
else
NUTTXLIBS	+= $(APPDIR)/libapps$(LIBEXT)
endif
endif

# Add libraries for network support

ifeq ($(CONFIG_NET),y)
NUTTXLIBS	+= net/libnet$(LIBEXT)
endif

# Add libraries for file system support

ifeq ($(CONFIG_NFILE_DESCRIPTORS),0)
ifneq ($(CONFIG_NSOCKET_DESCRIPTORS),0)
NUTTXLIBS	+= fs/libfs$(LIBEXT)
endif
ifeq ($(CONFIG_NET),y)
NUTTXLIBS	+= drivers/libdrivers$(LIBEXT)
endif
else
NUTTXLIBS	+= fs/libfs$(LIBEXT) drivers/libdrivers$(LIBEXT) binfmt/libbinfmt$(LIBEXT)
endif

# Add libraries for Pascall P-Code

ifneq ($(PCODE_DIR),)
NUTTXLIBS	+= $(PCODE_DIR)/libpcode$(LIBEXT)
endif

# Add libraries for the NX graphics sub-system

ifneq ($(NX_DIR),)
NUTTXLIBS	+= $(NX_DIR)/libnx$(LIBEXT)
endif

ifeq ($(CONFIG_NX),y)
NUTTXLIBS        += graphics/libgraphics$(LIBEXT)
endif

# This is the name of the final target
BIN		= nuttx$(EXEEXT)

all: $(BIN)
.PHONY: context clean_context check_context subdir_clean clean subdir_distclean distclean

# Build the mkconfig tool used to create include/nuttx/config.h
tools/mkconfig:
	@$(MAKE) -C tools -f Makefile.host TOPDIR="$(TOPDIR)"  mkconfig

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

syscall/libstubs$(LIBEXT): context
	@$(MAKE) -C syscall TOPDIR="$(TOPDIR)" libstubs$(LIBEXT)

syscall/libproxies$(LIBEXT): context
	@$(MAKE) -C syscall TOPDIR="$(TOPDIR)" libproxies$(LIBEXT)

# If the 2 pass build option is selected, then this pass1 target is
# configured to built before the pass2 target.  This pass1 target may, as an
# example, build an extra link object (CONFIG_PASS1_OBJECT) which may be an
# incremental (relative) link object, but could be a static library (archive);
# some modification to this Makefile would be required if CONFIG_PASS1_OBJECT
# is an archive.  Exactly what is performed during pass1 or what it generates
# is unknown to this makefule unless CONFIG_PASS1_OBJECT is defined.

pass1deps: context pass1dep $(USERLIBS)

pass1: pass1deps
ifeq ($(CONFIG_BUILD_2PASS),y)
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
	@$(MAKE) -C $(CONFIG_PASS1_BUILDIR) TOPDIR="$(TOPDIR)" LINKLIBS="$(NUTTXLIBS)" USERLIBS="$(USERLIBS)" "$(CONFIG_PASS1_TARGET)"
endif

pass2deps: context pass2dep $(NUTTXLIBS)

pass2: pass2deps
	@$(MAKE) -C $(ARCH_SRC) TOPDIR="$(TOPDIR)" EXTRA_OBJS="$(EXTRA_OBJS)" LINKLIBS="$(NUTTXLIBS)" $(BIN)
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

# In the normal case, all pass1 and pass2 dependencies are created then pass1
# and pass2 targets are built.  However, in some cases, you may need to build
# pass1 depenencies and pass1 first, then build pass2 dependencies and pass2.
# in that case, execute 'make pass1 pass2' from the command line.

$(BIN): pass1deps pass2deps pass1 pass2

# This is a helper target that will rebuild NuttX and download it to the
# target system in one step.  It will generate an error an error if the
# DOWNLOAD command is not defined in platform Make.defs file.

download: $(BIN)
	$(call DOWNLOAD, $<)

pass1dep: context
	@for dir in $(USERDEPDIRS) ; do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" depend ; \
	done

pass2dep: context
	@for dir in $(KERNDEPDIRS) ; do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" depend ; \
	done

depend: pass1dep pass2dep

subdir_clean:
	@for dir in $(CLEANDIRS) ; do \
		if [ -e $$dir/Makefile ]; then \
			$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" clean ; \
		fi \
	done
	@$(MAKE) -C tools -f Makefile.host TOPDIR="$(TOPDIR)" clean
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


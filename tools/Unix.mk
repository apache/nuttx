############################################################################
# tools/Unix.mk
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

export TOPDIR := ${shell echo $(CURDIR) | sed -e 's/ /\\ /g'}

ifeq ($(V),)
  MAKE := $(MAKE) -s --no-print-directory
endif

# Build any necessary tools needed early in the build.
# incdir - Is needed immediately by all Make.defs file.

DUMMY  := ${shell $(MAKE) -C tools -f Makefile.host incdir \
          INCDIR="$(TOPDIR)/tools/incdir.sh"}

include $(TOPDIR)/Make.defs

# GIT directory present

GIT_DIR = $(if $(wildcard $(TOPDIR)$(DELIM).git),y,)

ifeq ($(GIT_DIR),y)
GIT_PRESENT = `git rev-parse --git-dir 2> /dev/null`
endif

# In case we cannot get version information from GIT

ifeq ($(GIT_PRESENT),)
-include $(TOPDIR)/.version

# In case the version file does not exist

CONFIG_VERSION_STRING ?= "0.0.0"
CONFIG_VERSION_BUILD ?= "0"

VERSION_ARG = -v $(CONFIG_VERSION_STRING) -b $(CONFIG_VERSION_BUILD)
else

# Generate .version.tmp every time from GIT history
# Only update .version if the contents of version.tmp actually changes
# Note: this is executed before any rule is run

$(shell tools/version.sh .version.tmp > /dev/null)
$(shell $(call TESTANDREPLACEFILE, .version.tmp, .version))
endif

# Process architecture specific directories

ARCH_DIR = arch/$(CONFIG_ARCH)
ARCH_SRC = $(ARCH_DIR)/src
ARCH_INC = $(ARCH_DIR)/include

# CONFIG_APPS_DIR can be over-ridden from the command line or in the .config file.
# The default value of CONFIG_APPS_DIR is ../apps.  Ultimately, the application
# will be built if APPDIR is defined.  APPDIR will be defined if a directory containing
# a Makefile is found at the path provided by CONFIG_APPS_DIR

ifeq ($(CONFIG_APPS_DIR),)
CONFIG_APPS_DIR = ../apps
endif
APPDIR := $(realpath ${shell if [ -r $(CONFIG_APPS_DIR)/Makefile ]; then echo "$(CONFIG_APPS_DIR)"; fi})

# External code support
# If external/ contains a Kconfig, we define the EXTERNALDIR variable to 'external'
# so that main Kconfig can find it. Otherwise, we redirect it to a dummy Kconfig
# This is due to kconfig inability to do conditional inclusion.

EXTERNALDIR := $(shell if [ -r $(TOPDIR)/external/Kconfig ]; then echo 'external'; else echo 'dummy'; fi)

# CONTEXTDIRS include directories that have special, one-time pre-build
#   requirements.  Normally this includes things like auto-generation of
#   configuration specific files or creation of configurable symbolic links
# CLEANDIRS are the directories that the clean target will executed in.
#   These are all directories that we know about.
# CCLEANDIRS are directories that the clean_context target will execute in.
#   The clean_context target "undoes" the actions of the context target.
#   Only directories known to require cleaning are included.
# KERNDEPDIRS are the directories in which we will build target dependencies.
#   If NuttX and applications are built separately (CONFIG_BUILD_PROTECTED or
#   CONFIG_BUILD_KERNEL), then this holds only the directories containing
#   kernel files.
# USERDEPDIRS. If NuttX and applications are built separately (CONFIG_BUILD_PROTECTED),
#   then this holds only the directories containing user files. If
#   CONFIG_BUILD_KERNEL is selected, then applications are not build at all.

include tools/Directories.mk

#
# Extra objects used in the final link.
#
# Pass 1 Incremental (relative) link objects should be put into the
# processor-specific source directory (where other link objects will
# be created).  If the pass1 object is an archive, it could go anywhere.

ifeq ($(CONFIG_BUILD_2PASS),y)
EXTRA_OBJS += $(CONFIG_PASS1_OBJECT)
endif

# Library build selections
#
# NUTTXLIBS is the list of NuttX libraries that is passed to the
#   processor-specific Makefile to build the final NuttX target.
# USERLIBS is the list of libraries used to build the final user-space
#   application
# EXPORTLIBS is the list of libraries that should be exported by
#   'make export' is

ifeq ($(CONFIG_BUILD_PROTECTED),y)
include tools/ProtectedLibs.mk
else ifeq ($(CONFIG_BUILD_KERNEL),y)
include tools/KernelLibs.mk
else
include tools/FlatLibs.mk
endif

# LINKLIBS derives from NUTTXLIBS and is simply the same list with the
#   subdirectory removed

LINKLIBS = $(patsubst staging/%,%,$(NUTTXLIBS))

# Export tool definitions

MKEXPORT= tools/mkexport.sh
MKEXPORT_ARGS = -z -t "$(TOPDIR)" -b "$(BOARD_DIR)"

ifneq ($(CONFIG_BUILD_FLAT),y)
MKEXPORT_ARGS += -u
endif

ifneq ($(APPDIR),)
ifneq ($(shell [ -e $(APPDIR)/Makefile ] && echo yes),)
MKEXPORT_ARGS += -a "$(APPDIR)"
MKEXPORT_ARGS += -m "$(MAKE)"
endif
endif

ifeq ($(V),2)
MKEXPORT_ARGS += -d
endif

# This is the name of the final target (relative to the top level directory)

BIN = nuttx$(EXEEXT)

all: $(BIN)
.PHONY: context clean_context config oldconfig menuconfig nconfig qconfig gconfig export subdir_clean clean subdir_distclean distclean apps_clean apps_distclean
.PHONY: pass1 pass1dep
.PHONY: pass2 pass2dep

# Target used to copy include/nuttx/lib/math.h.  If CONFIG_ARCH_MATH_H is
# defined, then there is an architecture specific math.h header file
# that will be included indirectly from include/math.h.  But first, we
# have to copy math.h from include/nuttx/. to include/.  Logic within
# include/nuttx/lib/math.h will hand the redirection to the architecture-
# specific math.h header file.
#
# If the CONFIG_LIBM is defined, the Rhombus libm will be built at libc/math.
# Definitions and prototypes for the Rhombus libm are also contained in
# include/nuttx/lib/math.h and so the file must also be copied in that case.
#
# If neither CONFIG_ARCH_MATH_H nor CONFIG_LIBM is defined, then no math.h
# header file will be provided.  You would want that behavior if (1) you
# don't use libm, or (2) you want to use the math.h and libm provided
# within your toolchain.

ifeq ($(CONFIG_ARCH_MATH_H),y)
NEED_MATH_H = y
else ifeq ($(CONFIG_LIBM),y)
NEED_MATH_H = y
endif

ifeq ($(NEED_MATH_H),y)
include/math.h: include/nuttx/lib/math.h
	$(Q) cp -f include/nuttx/lib/math.h include/math.h
endif

# The float.h header file defines the properties of your floating point
# implementation.  It would always be best to use your toolchain's float.h
# header file but if none is available, a default float.h header file will
# provided if this option is selected.  However there is no assurance that
# the settings in this float.h are actually correct for your platform!

ifeq ($(CONFIG_ARCH_FLOAT_H),y)
include/float.h: include/nuttx/lib/float.h
	$(Q) cp -f include/nuttx/lib/float.h include/float.h
endif

# Target used to copy include/nuttx/lib/stdarg.h.  If CONFIG_ARCH_STDARG_H is
# defined, then there is an architecture specific stdarg.h header file
# that will be included indirectly from include/lib/stdarg.h.  But first, we
# have to copy stdarg.h from include/nuttx/. to include/.

ifeq ($(CONFIG_ARCH_STDARG_H),y)
include/stdarg.h: include/nuttx/lib/stdarg.h
	$(Q) cp -f include/nuttx/lib/stdarg.h include/stdarg.h
endif

# Target used to copy include/nuttx/lib/setjmp.h.  If CONFIG_ARCH_SETJMP_H is
# defined, then there is an architecture specific setjmp.h header file
# that will be included indirectly from include/lib/setjmp.h.  But first, we
# have to copy setjmp.h from include/nuttx/. to include/.

ifeq ($(CONFIG_ARCH_SETJMP_H),y)
include/setjmp.h: include/nuttx/lib/setjmp.h
	$(Q) cp -f include/nuttx/lib/setjmp.h include/setjmp.h
endif

# Targets used to build include/nuttx/version.h.  Creation of version.h is
# part of the overall NuttX configuration sequence. Notice that the
# tools/mkversion tool is built and used to create include/nuttx/version.h

tools/mkversion$(HOSTEXEEXT):
	$(Q) $(MAKE) -C tools -f Makefile.host mkversion$(HOSTEXEEXT)

# [Re-]create .version if it doesn't already exist.

$(TOPDIR)/.version:
	$(Q) echo "Create .version"
	$(Q) tools/version.sh $(VERSION_ARG) .version
	$(Q) chmod 755 .version

include/nuttx/version.h: $(TOPDIR)/.version tools/mkversion$(HOSTEXEEXT)
	$(Q) echo "Create version.h"
	$(Q) tools/mkversion $(TOPDIR) > $@

# Targets used to build include/nuttx/config.h.  Creation of config.h is
# part of the overall NuttX configuration sequence. Notice that the
# tools/mkconfig tool is built and used to create include/nuttx/config.h

tools/mkconfig$(HOSTEXEEXT):
	$(Q) $(MAKE) -C tools -f Makefile.host mkconfig$(HOSTEXEEXT)

include/nuttx/config.h: $(TOPDIR)/.config tools/mkconfig$(HOSTEXEEXT)
	$(Q) tools/mkconfig $(TOPDIR) > $@.tmp
	$(Q) $(call TESTANDREPLACEFILE, $@.tmp, $@)

# Targets used to create dependencies

tools/mkdeps$(HOSTEXEEXT):
	$(Q) $(MAKE) -C tools -f Makefile.host mkdeps$(HOSTEXEEXT)

tools/cnvwindeps$(HOSTEXEEXT):
	$(Q) $(MAKE) -C tools -f Makefile.host cnvwindeps$(HOSTEXEEXT)

# .dirlinks, and helpers
#
# Directories links.  Most of establishing the NuttX configuration involves
# setting up symbolic links with 'generic' directory names to specific,
# configured directories.

# Link the arch/<arch-name>/include directory to include/arch

include/arch:
	@echo "LN: $@ to $(ARCH_DIR)/include"
	$(Q) $(DIRLINK) $(TOPDIR)/$(ARCH_DIR)/include $@

# Link the boards/<arch>/<chip>/<board>/include directory to include/arch/board

include/arch/board: | include/arch
	@echo "LN: $@ to $(BOARD_DIR)/include"
	$(Q) $(DIRLINK) $(BOARD_DIR)/include $@

# Link the boards/<arch>/<chip>/common dir to arch/<arch-name>/src/board
# Link the boards/<arch>/<chip>/<board>/src dir to arch/<arch-name>/src/board/board

ifneq ($(BOARD_COMMON_DIR),)
ARCH_SRC_BOARD_SYMLINK=$(BOARD_COMMON_DIR)
ARCH_SRC_BOARD_BOARD_SYMLINK=$(BOARD_DIR)/src
else
ARCH_SRC_BOARD_SYMLINK=$(BOARD_DIR)/src
endif

ifneq ($(ARCH_SRC_BOARD_SYMLINK),)
$(ARCH_SRC)/board:
	@echo "LN: $@ to $(ARCH_SRC_BOARD_SYMLINK)"
	$(Q) $(DIRLINK) $(ARCH_SRC_BOARD_SYMLINK) $@
endif

ifneq ($(ARCH_SRC_BOARD_BOARD_SYMLINK),)
$(ARCH_SRC)/board/board: | $(ARCH_SRC)/board
	@echo "LN: $@ to $(ARCH_SRC_BOARD_BOARD_SYMLINK)"
	$(Q) $(DIRLINK) $(ARCH_SRC_BOARD_BOARD_SYMLINK) $@
endif

# Link the boards/<arch>/<chip>/drivers dir to drivers/platform

drivers/platform:
	@echo "LN: $@ to $(BOARD_DRIVERS_DIR)"
	$(Q) $(DIRLINK) $(BOARD_DRIVERS_DIR) $@

# Link arch/<arch-name>/src/<chip-name> to arch/<arch-name>/src/chip

ifeq ($(CONFIG_ARCH_CHIP_CUSTOM),y)
ARCH_SRC_CHIP_SYMLINK_DIR=$(CHIP_DIR)
else ifneq ($(CONFIG_ARCH_CHIP),)
ARCH_SRC_CHIP_SYMLINK_DIR=$(TOPDIR)/$(ARCH_SRC)/$(CONFIG_ARCH_CHIP)
endif

ifneq ($(ARCH_SRC_CHIP_SYMLINK_DIR),)
$(ARCH_SRC)/chip:
	@echo "LN: $@ to $(ARCH_SRC_CHIP_SYMLINK_DIR)"
	$(Q) $(DIRLINK) $(ARCH_SRC_CHIP_SYMLINK_DIR) $@
endif

# Link arch/<arch-name>/include/<chip-name> to include/arch/chip

ifeq ($(CONFIG_ARCH_CHIP_CUSTOM),y)
INCLUDE_ARCH_CHIP_SYMLINK_DIR=$(CHIP_DIR)/include
else ifneq ($(CONFIG_ARCH_CHIP),)
INCLUDE_ARCH_CHIP_SYMLINK_DIR=$(TOPDIR)/$(ARCH_INC)/$(CONFIG_ARCH_CHIP)
endif

ifneq ($(INCLUDE_ARCH_CHIP_SYMLINK_DIR),)
include/arch/chip: | include/arch
	@echo "LN: $@ to $(INCLUDE_ARCH_CHIP_SYMLINK_DIR)"
	$(DIRLINK) $(INCLUDE_ARCH_CHIP_SYMLINK_DIR) $@
endif

# Copy $(CHIP_KCONFIG) to arch/dummy/Kconfig

arch/dummy/Kconfig:
	@echo "CP: $@ to $(CHIP_KCONFIG)"
	$(Q) cp -f $(CHIP_KCONFIG) $@

# Copy $(BOARD_KCONFIG) to boards/dummy/Kconfig

boards/dummy/Kconfig:
	@echo "CP: $@ to $(BOARD_KCONFIG)"
	$(Q) cp -f $(BOARD_KCONFIG) $@

DIRLINKS_SYMLINK = \
  include/arch \
  include/arch/board \
  drivers/platform \

DIRLINKS_FILE = \
  arch/dummy/Kconfig \
  boards/dummy/Kconfig \

ifneq ($(INCLUDE_ARCH_CHIP_SYMLINK_DIR),)
DIRLINKS_SYMLINK += include/arch/chip
endif

ifneq ($(ARCH_SRC_CHIP_SYMLINK_DIR),)
DIRLINKS_SYMLINK += $(ARCH_SRC)/chip
endif

ifneq ($(ARCH_SRC_BOARD_SYMLINK),)
DIRLINKS_SYMLINK += $(ARCH_SRC)/board
endif

ifneq ($(ARCH_SRC_BOARD_BOARD_SYMLINK),)
DIRLINKS_SYMLINK += $(ARCH_SRC)/board/board
endif

ifneq ($(APPDIR),)
DIRLINKS_EXTERNAL_DIRS += $(APPDIR)
endif

# Generate a pattern to build $(DIRLINKS_EXTERNAL_DIRS)

DIRLINKS_EXTERNAL_DEP = $(patsubst %,%/.dirlinks,$(DIRLINKS_EXTERNAL_DIRS))
DIRLINKS_FILE += $(DIRLINKS_EXTERNAL_DEP)

.dirlinks: $(DIRLINKS_FILE) | $(DIRLINKS_SYMLINK)
	touch $@

# Pattern rule for $(DIRLINKS_EXTERNAL_DEP)

%/.dirlinks:
	$(Q) $(MAKE) -C $(patsubst %/.dirlinks,%,$@) dirlinks
	$(Q) touch $@

# clean_dirlinks
#
# This is part of the distclean target. It removes all symbolic links created by the dirlink target.

# The symlink subfolders need to be removed before the parent symlinks

.PHONY: clean_dirlinks
clean_dirlinks:
	$(Q) $(call DELFILE, $(DIRLINKS_FILE))
	$(Q) $(call DELFILE, .dirlinks)
	$(Q) $(DIRUNLINK) drivers/platform
ifneq ($(INCLUDE_ARCH_CHIP_SYMLINK_DIR),)
	$(Q) $(DIRUNLINK) include/arch/chip
endif
	$(Q) $(DIRUNLINK) include/arch/board
	$(Q) $(DIRUNLINK) include/arch
ifneq ($(ARCH_SRC_BOARD_BOARD_SYMLINK),)
	$(Q) $(DIRUNLINK) $(ARCH_SRC)/board/board
endif
ifneq ($(ARCH_SRC_BOARD_SYMLINK),)
	$(Q) $(DIRUNLINK) $(ARCH_SRC)/board
endif
ifneq ($(ARCH_SRC_CHIP_SYMLINK_DIR),)
	$(Q) $(DIRUNLINK) $(ARCH_SRC)/chip
endif

# context
#
# The context target is invoked on each target build to assure that NuttX is
# properly configured.  The basic configuration steps include creation of the
# the config.h and version.h header files in the include/nuttx directory and
# the establishment of symbolic links to configured directories.

# Generate a pattern to make Directories.mk context

CONTEXTDIRS_DEPS = $(patsubst %,%/.context,$(CONTEXTDIRS))

context: include/nuttx/config.h include/nuttx/version.h .dirlinks $(CONTEXTDIRS_DEPS) | staging

staging:
	$(Q) mkdir -p $@

# Pattern rule for $(CONTEXTDIRS_DEPS)

%.context: include/nuttx/config.h .dirlinks
	$(Q) $(MAKE) -C $(patsubst %.context,%,$@) TOPDIR="$(TOPDIR)" context
	$(Q) touch $@

ifeq ($(NEED_MATH_H),y)
context: include/math.h
endif

ifeq ($(CONFIG_ARCH_FLOAT_H),y)
context: include/float.h
endif

ifeq ($(CONFIG_ARCH_STDARG_H),y)
context: include/stdarg.h
endif

ifeq ($(CONFIG_ARCH_SETJMP_H),y)
context: include/setjmp.h
endif

# clean_context
#
# This is part of the distclean target.  It removes all of the header files
# and symbolic links created by the context target.

clean_context: clean_dirlinks
	$(Q) for dir in $(CCLEANDIRS) ; do \
		if [ -e $$dir/Makefile ]; then \
			$(MAKE) -C $$dir clean_context ; \
		fi \
	done
	$(call DELFILE, include/nuttx/config.h)
	$(call DELFILE, include/nuttx/version.h)
	$(call DELFILE, include/float.h)
	$(call DELFILE, include/math.h)
	$(call DELFILE, include/stdarg.h)
	$(call DELFILE, include/setjmp.h)
	$(call DELFILE, arch/dummy/Kconfig)
	$(call DELFILE, $(CONTEXTDIRS_DEPS))

# Archive targets.  The target build sequence will first create a series of
# libraries, one per configured source file directory.  The final NuttX
# execution will then be built from those libraries.  The following targets
# build those libraries.

include tools/LibTargets.mk

# pass1 and pass2
#
# If the 2 pass build option is selected, then this pass1 target is
# configured to be built before the pass2 target.  This pass1 target may, as an
# example, build an extra link object (CONFIG_PASS1_OBJECT) which may be an
# incremental (relative) link object, but could be a static library (archive);
# some modification to this Makefile would be required if CONFIG_PASS1_OBJECT
# is an archive.  Exactly what is performed during pass1 or what it generates
# is unknown to this makefile unless CONFIG_PASS1_OBJECT is defined.

pass1: $(USERLIBS)

pass2: $(NUTTXLIBS)

# $(BIN)
#
# Create the final NuttX executable in a two pass build process.  In the
# normal case, all pass1 and pass2 dependencies are created then pass1
# and pass2 targets are built.  However, in some cases, you may need to build
# pass1 dependencies and pass1 first, then build pass2 dependencies and pass2.
# in that case, execute 'make pass1 pass2' from the command line.

$(BIN): pass1 pass2
ifeq ($(CONFIG_BUILD_2PASS),y)
	$(Q) if [ -z "$(CONFIG_PASS1_BUILDIR)" ]; then \
		echo "ERROR: CONFIG_PASS1_BUILDIR not defined"; \
		exit 1; \
	fi
	$(Q) if [ ! -d "$(CONFIG_PASS1_BUILDIR)" ]; then \
		echo "ERROR: CONFIG_PASS1_BUILDIR does not exist"; \
		exit 1; \
	fi
	$(Q) if [ ! -f "$(CONFIG_PASS1_BUILDIR)/Makefile" ]; then \
		echo "ERROR: No Makefile in CONFIG_PASS1_BUILDIR"; \
		exit 1; \
	fi
	$(Q) $(MAKE) -C $(CONFIG_PASS1_BUILDIR) LINKLIBS="$(LINKLIBS)" USERLIBS="$(USERLIBS)" "$(CONFIG_PASS1_TARGET)"
endif
	$(Q) $(MAKE) -C $(ARCH_SRC) EXTRA_OBJS="$(EXTRA_OBJS)" LINKLIBS="$(LINKLIBS)" APPDIR="$(APPDIR)" EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)" $(BIN)
	$(Q) if [ -w /tftpboot ] ; then \
		cp -f $(BIN) /tftpboot/$(BIN).${CONFIG_ARCH}; \
	fi
	$(Q) echo $(BIN) > nuttx.manifest
	$(Q) printf "%s\n" *.map >> nuttx.manifest
ifeq ($(CONFIG_INTELHEX_BINARY),y)
	@echo "CP: nuttx.hex"
	$(Q) $(OBJCOPY) $(OBJCOPYARGS) -O ihex $(BIN) nuttx.hex
	$(Q) echo nuttx.hex >> nuttx.manifest
endif
ifeq ($(CONFIG_MOTOROLA_SREC),y)
	@echo "CP: nuttx.srec"
	$(Q) $(OBJCOPY) $(OBJCOPYARGS) -O srec $(BIN) nuttx.srec
	$(Q) echo nuttx.srec >> nuttx.manifest
endif
ifeq ($(CONFIG_RAW_BINARY),y)
	@echo "CP: nuttx.bin"
	$(Q) $(OBJCOPY) $(OBJCOPYARGS) -O binary $(BIN) nuttx.bin
	$(Q) echo nuttx.bin >> nuttx.manifest
endif
ifeq ($(CONFIG_UBOOT_UIMAGE),y)
	@echo "MKIMAGE: uImage"
	$(Q) mkimage -A $(CONFIG_ARCH) -O linux -C none -T kernel -a $(CONFIG_UIMAGE_LOAD_ADDRESS) \
		-e $(CONFIG_UIMAGE_ENTRY_POINT) -n $(BIN) -d nuttx.bin uImage
	$(Q) if [ -w /tftpboot ] ; then \
		cp -f uImage /tftpboot/uImage; \
	fi
	$(Q) echo "uImage" >> nuttx.manifest
endif
	$(call POSTBUILD, $(TOPDIR))

# flash (or download : DEPRECATED)
#
# This is a helper target that will rebuild NuttX and flash it to the target
# system in one step.  The operation of this target depends completely upon
# implementation of the FLASH command in the user Make.defs file.  It will
# generate an error if the FLASH command is not defined.

flash: $(BIN)
	$(call FLASH, $<)

download: $(BIN)
	$(call FLASH, $<)

# bootloader
#
# Some architectures require the provisioning of a bootloader or other
# functions required for properly executing the NuttX binary.
# Make.defs files for those architectures should define a bootloader target
# with the correct operations for that platform. It will generate an error
# if the bootloader target is not defined.

bootloader:
	$(Q) $(MAKE) -C $(ARCH_SRC) bootloader

# clean_bootloader
#
# Removes all of the files and directories created by the bootloader target.
# It will generate an error if the clean_bootloader target is not defined on
# the architecture-specific Makefile.

clean_bootloader:
	$(Q) $(MAKE) -C $(ARCH_SRC) clean_bootloader

# pass1dep: Create pass1 build dependencies
# pass2dep: Create pass2 build dependencies

pass1dep: context tools/mkdeps$(HOSTEXEEXT) tools/cnvwindeps$(HOSTEXEEXT)
	$(Q) for dir in $(USERDEPDIRS) ; do \
		$(MAKE) -C $$dir depend || exit; \
	done

pass2dep: context tools/mkdeps$(HOSTEXEEXT) tools/cnvwindeps$(HOSTEXEEXT)
	$(Q) for dir in $(KERNDEPDIRS) ; do \
		$(MAKE) -C $$dir EXTRAFLAGS="$(KDEFINE) $(EXTRAFLAGS)" depend || exit; \
	done

# Configuration targets
#
# These targets depend on the kconfig-frontends packages.  To use these, you
# must first download and install the kconfig-frontends package from this
# location: https://bitbucket.org/nuttx/tools/downloads/.  See README.txt
# file in the NuttX tools GIT repository for additional information.

KCONFIG_ENV  = APPSDIR=${CONFIG_APPS_DIR} EXTERNALDIR=$(EXTERNALDIR)
KCONFIG_ENV += APPSBINDIR=${CONFIG_APPS_DIR} BINDIR=${TOPDIR}

LOADABLE = $(shell grep "=m$$" $(TOPDIR)/.config)
ifeq ($(CONFIG_BUILD_LOADABLE)$(LOADABLE),)
  KCONFIG_LIB = $(shell command -v menuconfig 2> /dev/null)
endif

# Prefer "kconfiglib" if host OS supports it

ifeq ($(KCONFIG_LIB),)
  KCONFIG_OLDCONFIG     = kconfig-conf --oldconfig Kconfig
  KCONFIG_OLDDEFCONFIG  = kconfig-conf --olddefconfig Kconfig
  KCONFIG_MENUCONFIG    = kconfig-mconf Kconfig
  KCONFIG_NCONFIG       = kconfig-nconf Kconfig
  KCONFIG_QCONFIG       = kconfig-qconf Kconfig
  KCONFIG_GCONFIG       = kconfig-gconf Kconfig
  KCONFIG_SAVEDEFCONFIG = kconfig-conf Kconfig --savedefconfig defconfig.tmp
define kconfig_tweak_disable
	kconfig-tweak --file $1 -u $2
endef
else
  PURGE_MODULE_WARNING  = 2>&1 | grep -v "warning: the 'modules' option is not supported"
  KCONFIG_OLDCONFIG     = oldconfig ${PURGE_MODULE_WARNING}
  KCONFIG_OLDDEFCONFIG  = olddefconfig ${PURGE_MODULE_WARNING}
  KCONFIG_MENUCONFIG    = menuconfig ${PURGE_MODULE_WARNING}
  KCONFIG_NCONFIG       = guiconfig ${PURGE_MODULE_WARNING}
  KCONFIG_QCONFIG       = ${KCONFIG_NCONFIG}
  KCONFIG_GCONFIG       = ${KCONFIG_NCONFIG}
  KCONFIG_SAVEDEFCONFIG = savedefconfig --out defconfig.tmp ${PURGE_MODULE_WARNING}
define kconfig_tweak_disable
	$(Q) sed -i'.orig' '/$2/d' $1
	$(Q) rm -f $1.orig
endef
endif

KCONFIG_CONF = kconfig-conf

config:
	$(Q) $(MAKE) clean_context
	$(Q) $(MAKE) apps_preconfig
	$(Q) ${KCONFIG_ENV} ${KCONFIG_CONF}

oldconfig:
	$(Q) $(MAKE) clean_context
	$(Q) $(MAKE) apps_preconfig
	$(Q) ${KCONFIG_ENV} ${KCONFIG_OLDCONFIG}

olddefconfig:
	$(Q) $(MAKE) clean_context
	$(Q) $(MAKE) apps_preconfig
	$(Q) ${KCONFIG_ENV} ${KCONFIG_OLDDEFCONFIG}

menuconfig:
	$(Q) $(MAKE) clean_context
	$(Q) $(MAKE) apps_preconfig
	$(Q) ${KCONFIG_ENV} ${KCONFIG_MENUCONFIG}

nconfig: apps_preconfig
	$(Q) $(MAKE) clean_context
	$(Q) $(MAKE) apps_preconfig
	$(Q) ${KCONFIG_ENV} ${KCONFIG_NCONFIG}

qconfig: apps_preconfig
	$(Q) $(MAKE) clean_context
	$(Q) $(MAKE) apps_preconfig
	$(Q) ${KCONFIG_ENV} ${KCONFIG_QCONFIG}

gconfig: apps_preconfig
	$(Q) $(MAKE) clean_context
	$(Q) $(MAKE) apps_preconfig
	$(Q) ${KCONFIG_ENV} ${KCONFIG_GCONFIG}

savedefconfig: apps_preconfig
	$(Q) ${KCONFIG_ENV} ${KCONFIG_SAVEDEFCONFIG}
	$(Q) $(call kconfig_tweak_disable,defconfig.tmp,CONFIG_APPS_DIR)
	$(Q) grep "CONFIG_ARCH=" .config >> defconfig.tmp
	$(Q) grep "^CONFIG_ARCH_CHIP_" .config >> defconfig.tmp; true
	$(Q) grep "CONFIG_ARCH_CHIP=" .config >> defconfig.tmp; true
	$(Q) grep "CONFIG_ARCH_BOARD=" .config >> defconfig.tmp; true
	$(Q) grep "^CONFIG_ARCH_CUSTOM" .config >> defconfig.tmp; true
	$(Q) grep "^CONFIG_ARCH_BOARD_CUSTOM" .config >> defconfig.tmp; true
	$(Q) export LC_ALL=C; cat defconfig.tmp | sort | uniq > sortedconfig.tmp
	$(Q) echo "#" > warning.tmp
	$(Q) echo "# This file is autogenerated: PLEASE DO NOT EDIT IT." >> warning.tmp
	$(Q) echo "#" >> warning.tmp
	$(Q) echo "# You can use \"make menuconfig\" to make any modifications to the installed .config file." >> warning.tmp
	$(Q) echo "# You can then do \"make savedefconfig\" to generate a new defconfig file that includes your" >> warning.tmp
	$(Q) echo "# modifications." >> warning.tmp
	$(Q) echo "#" >> warning.tmp
	$(Q) cat warning.tmp sortedconfig.tmp > defconfig
	$(Q) rm -f warning.tmp
	$(Q) rm -f defconfig.tmp
	$(Q) rm -f sortedconfig.tmp

# export
#
# The export target will package the NuttX libraries and header files into
# an exportable package.  Caveats: (1) These needs some extension for the KERNEL
# build; it needs to receive USERLIBS and create a libuser.a). (2) The logic
# in tools/mkexport.sh only supports GCC and, for example, explicitly assumes
# that the archiver is 'ar'

export: $(NUTTXLIBS)
	$(Q) MAKE="${MAKE}" $(MKEXPORT) $(MKEXPORT_ARGS) -l "$(EXPORTLIBS)"

# General housekeeping targets:  dependencies, cleaning, etc.
#
# depend:    Create both PASS1 and PASS2 dependencies
# clean:     Removes derived object files, archives, executables, and
#            temporary files, but retains the configuration and context
#            files and directories.
# distclean: Does 'clean' then also removes all configuration and context
#            files.  This essentially restores the directory structure
#            to its original, unconfigured stated.

depend: pass1dep pass2dep

$(foreach SDIR, $(CLEANDIRS), $(eval $(call SDIR_template,$(SDIR),clean)))

subdir_clean: $(foreach SDIR, $(CLEANDIRS), $(SDIR)_clean)
ifeq ($(CONFIG_BUILD_2PASS),y)
	$(Q) $(MAKE) -C $(CONFIG_PASS1_BUILDIR) clean
endif

clean: subdir_clean
	$(call DELFILE, $(BIN))
	$(call DELFILE, nuttx.*)
	$(call DELFILE, *.map)
	$(call DELFILE, _SAVED_APPS_config)
	$(call DELFILE, nuttx-export*.zip)
	$(call DELDIR, nuttx-export*)
	$(call DELFILE, nuttx_user*)
	$(call DELDIR, staging)
	$(call DELFILE, uImage)
	$(call CLEAN)

$(foreach SDIR, $(CLEANDIRS), $(eval $(call SDIR_template,$(SDIR),distclean)))

subdir_distclean: $(foreach SDIR, $(CLEANDIRS), $(SDIR)_distclean)

distclean: clean subdir_distclean
ifeq ($(CONFIG_BUILD_2PASS),y)
	$(Q) $(MAKE) -C $(CONFIG_PASS1_BUILDIR) distclean
endif
ifeq ($(CONFIG_ARCH_HAVE_BOOTLOADER),y)
	$(Q) $(MAKE) clean_bootloader
endif
	$(Q) $(MAKE) clean_context
	$(Q) $(MAKE) -C tools -f Makefile.host clean
	$(call DELFILE, Make.defs)
	$(call DELFILE, defconfig)
	$(call DELFILE, .config)
	$(call DELFILE, .config.old)
	$(call DELFILE, .gdbinit)

# Application housekeeping targets.  The APPDIR variable refers to the user
# application directory.  A sample apps/ directory is included with NuttX,
# however, this is not treated as part of NuttX and may be replaced with a
# different application directory.  For the most part, the application
# directory is treated like any other build directory in this script.  However,
# as a convenience, the following targets are included to support housekeeping
# functions in the user application directory from the NuttX build directory.
#
# apps_preconfig: Prepare applications to be configured
# apps_clean:     Perform the clean operation only in the user application
#                 directory
# apps_distclean: Perform the distclean operation only in the user application
#                 directory.

apps_preconfig: .dirlinks
ifneq ($(APPDIR),)
	$(Q) $(MAKE) -C $(APPDIR) preconfig
endif

apps_clean:
ifneq ($(APPDIR),)
	$(Q) $(MAKE) -C $(APPDIR) clean
endif

apps_distclean:
ifneq ($(APPDIR),)
	$(Q) $(MAKE) -C $(APPDIR) distclean
endif

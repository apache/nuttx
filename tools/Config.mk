############################################################################
# tools/Config.mk
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

# Disable all built-in rules

.SUFFIXES:

# Control build verbosity
#
#  V=0:   Exit silent mode
#  V=1,2: Enable echo of commands
#  V=2:   Enable bug/verbose options in tools and scripts

ifeq ($(V),1)
export Q :=
else ifeq ($(V),2)
export Q :=
else
export Q := @
endif

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  export SHELL=cmd
else ifeq ($(V),)
  BASHCMD := $(shell command -v bash 2> /dev/null)
  ifneq ($(BASHCMD),)
    export SHELL=$(BASHCMD)
    export ECHO_BEGIN=@echo -ne "\033[1K\r"
    export ECHO_END=$(ECHO_BEGIN)
  endif
endif

ifeq ($(ECHO_BEGIN),)
  export ECHO_BEGIN=@echo
  export ECHO_END=
endif

# These are configuration variables that are quoted by configuration tool
# but which must be unquoted when used in the build system.

CONFIG_ARCH       := $(patsubst "%",%,$(strip $(CONFIG_ARCH)))
CONFIG_ARCH_CHIP  := $(patsubst "%",%,$(strip $(CONFIG_ARCH_CHIP)))
CONFIG_ARCH_BOARD := $(patsubst "%",%,$(strip $(CONFIG_ARCH_BOARD)))

# Some defaults.
# $(TOPDIR)/Make.defs can override these appropriately.

MODULECC ?= $(CC)
MODULELD ?= $(LD)
MODULESTRIP ?= $(STRIP)

# ccache configuration.

ifeq ($(CONFIG_CCACHE),y)
  CCACHE ?= ccache
endif

# Define HOSTCC on the make command line if it differs from these defaults
# Define HOSTCFLAGS with -g on the make command line to build debug versions

ifeq ($(CONFIG_WINDOWS_NATIVE),y)

# In the Windows native environment, the MinGW GCC compiler is used

HOSTCC ?= mingw32-gcc.exe
HOSTCFLAGS ?= -O2 -Wall -Wstrict-prototypes -Wshadow -DCONFIG_WINDOWS_NATIVE=y

else

# GCC or clang is assumed in all other POSIX environments
# (Linux, Cygwin, MSYS2, macOS).
# strtok_r is used in some tools, but does not seem to be available in
# the MinGW environment.

HOSTCC ?= cc
HOSTCFLAGS ?= -O2 -Wall -Wstrict-prototypes -Wshadow
HOSTCFLAGS += -DHAVE_STRTOK_C=1

ifeq ($(CONFIG_WINDOWS_CYGWIN),y)
HOSTCFLAGS += -DHOST_CYGWIN=1
endif
endif

# Some defaults just to prohibit some bad behavior if for some reason they
# are not defined

ASMEXT ?= .S
OBJEXT ?= .o
LIBEXT ?= .a

ifeq ($(CONFIG_WINDOWS_CYGWIN),y)
  EXEEXT ?= .exe
endif

ifeq ($(CONFIG_HOST_WINDOWS),y)
  HOSTEXEEXT ?= .exe
  HOSTDYNEXT ?= .dll
else ifeq ($(CONFIG_HOST_LINUX),y)
  HOSTDYNEXT ?= .so
endif

# This define is passed as EXTRAFLAGS for kernel-mode builds.  It is also passed
# during PASS1 (but not PASS2) context and depend targets.

KDEFINE ?= ${DEFINE_PREFIX}__KERNEL__

# DELIM - Path segment delimiter character
#
# Depends on this settings defined in board-specific defconfig file installed
# at $(TOPDIR)/.config:
#
#   CONFIG_WINDOWS_NATIVE - Defined for a Windows native build

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  DELIM ?= $(strip \)
else
  DELIM ?= $(strip /)
endif

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  EMPTYFILE := "NUL"
else
  EMPTYFILE := "/dev/null"
endif

# Process chip-specific directories

ifeq ($(CONFIG_ARCH_CHIP_CUSTOM),y)
  CUSTOM_CHIP_DIR = $(patsubst "%",%,$(CONFIG_ARCH_CHIP_CUSTOM_DIR))
ifeq ($(CONFIG_ARCH_CHIP_CUSTOM_DIR_RELPATH),y)
  CHIP_DIR ?= $(TOPDIR)$(DELIM)$(CUSTOM_CHIP_DIR)
  CHIP_KCONFIG = $(TOPDIR)$(DELIM)$(CUSTOM_CHIP_DIR)$(DELIM)Kconfig
else
  CHIP_DIR ?= $(CUSTOM_CHIP_DIR)
  CHIP_KCONFIG = $(CUSTOM_CHIP_DIR)$(DELIM)Kconfig
endif
else
  CHIP_DIR ?= $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src$(DELIM)$(CONFIG_ARCH_CHIP)
  CHIP_KCONFIG = $(TOPDIR)$(DELIM)arch$(DELIM)dummy$(DELIM)dummy_kconfig
endif

# Process board-specific directories

ifeq ($(CONFIG_ARCH_BOARD_CUSTOM),y)
  CUSTOM_DIR = $(patsubst "%",%,$(CONFIG_ARCH_BOARD_CUSTOM_DIR))
  ifeq ($(CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH),y)
    BOARD_DIR ?= $(TOPDIR)$(DELIM)$(CUSTOM_DIR)
  else
    BOARD_DIR ?= $(CUSTOM_DIR)
  endif
  CUSTOM_BOARD_KPATH = $(BOARD_DIR)$(DELIM)Kconfig
else
  BOARD_DIR ?= $(TOPDIR)$(DELIM)boards$(DELIM)$(CONFIG_ARCH)$(DELIM)$(CONFIG_ARCH_CHIP)$(DELIM)$(CONFIG_ARCH_BOARD)
endif
ifeq (,$(wildcard $(CUSTOM_BOARD_KPATH)))
  BOARD_KCONFIG = $(TOPDIR)$(DELIM)boards$(DELIM)dummy$(DELIM)dummy_kconfig
else
  BOARD_KCONFIG = $(CUSTOM_BOARD_KPATH)
endif

ifeq (,$(wildcard $(BOARD_DIR)$(DELIM)..$(DELIM)common))
  ifeq ($(CONFIG_ARCH_BOARD_COMMON),y)
    BOARD_COMMON_DIR ?= $(wildcard $(TOPDIR)$(DELIM)boards$(DELIM)$(CONFIG_ARCH)$(DELIM)$(CONFIG_ARCH_CHIP)$(DELIM)common)
  endif
else
  BOARD_COMMON_DIR ?= $(wildcard $(BOARD_DIR)$(DELIM)..$(DELIM)common)
endif
BOARD_DRIVERS_DIR ?= $(wildcard $(BOARD_DIR)$(DELIM)..$(DELIM)drivers)
ifeq ($(BOARD_DRIVERS_DIR),)
  BOARD_DRIVERS_DIR = $(TOPDIR)$(DELIM)drivers$(DELIM)dummy
endif

# DIRLINK - Create a directory link in the portable way

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
ifeq ($(CONFIG_WINDOWS_MKLINK),y)
  DIRLINK   ?= $(TOPDIR)$(DELIM)tools$(DELIM)link.bat
else
  DIRLINK   ?= $(TOPDIR)$(DELIM)tools$(DELIM)copydir.bat
endif
  DIRUNLINK ?= $(TOPDIR)$(DELIM)tools$(DELIM)unlink.bat
else
ifeq ($(CONFIG_CYGWIN_WINTOOL),y)
  DIRLINK   ?= $(TOPDIR)$(DELIM)tools$(DELIM)copydir.sh
else ifeq ($(CONFIG_WINDOWS_MSYS),y)
  DIRLINK   ?= $(TOPDIR)$(DELIM)tools$(DELIM)copydir.sh
else
  DIRLINK   ?= $(TOPDIR)$(DELIM)tools$(DELIM)link.sh
endif
  DIRUNLINK ?= $(TOPDIR)$(DELIM)tools$(DELIM)unlink.sh
endif

# MKDEP - Create the depend rule in the portable way

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  MKDEP ?= $(TOPDIR)$(DELIM)tools$(DELIM)mkdeps$(HOSTEXEEXT) --winnative
else ifeq ($(CONFIG_CYGWIN_WINTOOL),y)
  MKDEP ?= $(TOPDIR)$(DELIM)tools$(DELIM)mkwindeps.sh
else
  MKDEP ?= $(TOPDIR)$(DELIM)tools$(DELIM)mkdeps$(HOSTEXEEXT)
endif

# Per-file dependency generation rules

OBJPATH ?= .

%.dds: %.S
	$(Q) $(MKDEP) --obj-path $(OBJPATH) --obj-suffix $(OBJEXT) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $< > $@

%.ddc: %.c
	$(Q) $(MKDEP) --obj-path $(OBJPATH) --obj-suffix $(OBJEXT) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $< > $@

%.ddp: %.cpp
	$(Q) $(MKDEP) --obj-path $(OBJPATH) --obj-suffix $(OBJEXT) $(DEPPATH) "$(CXX)" -- $(CXXFLAGS) -- $< > $@

%.ddx: %.cxx
	$(Q) $(MKDEP) --obj-path $(OBJPATH) --obj-suffix $(OBJEXT) $(DEPPATH) "$(CXX)" -- $(CXXFLAGS) -- $< > $@

%.ddh: %.c
	$(Q) $(MKDEP) --obj-path $(OBJPATH) --obj-suffix $(OBJEXT) $(DEPPATH) "$(CC)" -- $(HOSTCFLAGS) -- $< > $@

# INCDIR - Convert a list of directory paths to a list of compiler include
#   directories
# Example: CFFLAGS += ${shell $(INCDIR) [options] "compiler" "dir1" "dir2" "dir2" ...}
#
# Note that the compiler string and each directory path string must quoted if
# they contain spaces or any other characters that might get mangled by the
# shell
#
# Depends on this setting passed as a make command line definition from the
# toplevel Makefile:
#
#   TOPDIR - The path to the top level NuttX directory in the form
#     appropriate for the current build environment
#
# Depends on this settings defined in board-specific defconfig file installed
# at $(TOPDIR)/.config:
#
#   CONFIG_WINDOWS_NATIVE - Defined for a Windows native build

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  DEFINE ?= $(TOPDIR)\tools\define.bat
  INCDIR ?= $(TOPDIR)\tools\incdir.bat
else ifeq ($(CONFIG_CYGWIN_WINTOOL),y)
  DEFINE ?= "$(TOPDIR)/tools/define.sh" -w
  INCDIR ?= "$(TOPDIR)/tools/incdir$(HOSTEXEEXT)" -w
else
  DEFINE ?= "$(TOPDIR)/tools/define.sh"
  INCDIR ?= "$(TOPDIR)/tools/incdir$(HOSTEXEEXT)"
endif

# PREPROCESS - Default macro to run the C pre-processor
# Example: $(call PREPROCESS, in-file, out-file)
#
# Depends on these settings defined in board-specific Make.defs file
# installed at $(TOPDIR)/Make.defs:
#
#   CPP - The command to invoke the C pre-processor
#   CPPFLAGS - Options to pass to the C pre-processor
#
# '<filename>.c_CPPFLAGS += <options>' may also be used, as an example, to
# change the options used with the single file <filename>.c (or
# <filename>.S)

define PREPROCESS
	$(ECHO_BEGIN)"CPP: $1->$2 "
	$(Q) $(CPP) $(CPPFLAGS) $($(strip $1)_CPPFLAGS) $1 -o $2
	$(ECHO_END)
endef

# COMPILE - Default macro to compile one C file
# Example: $(call COMPILE, in-file, out-file, flags)
#
# Depends on these settings defined in board-specific Make.defs file
# installed at $(TOPDIR)/Make.defs:
#
#   CC - The command to invoke the C compiler
#   CFLAGS - Options to pass to the C compiler
#
# '<filename>.c_CFLAGS += <options>' may also be used, as an example, to
# change the options used with the single file <filename>.c

define COMPILE
	$(ECHO_BEGIN)"CC: $1 "
	$(Q) $(CCACHE) $(CC) -c $(CFLAGS) $3 $($(strip $1)_CFLAGS) $1 -o $2
	$(ECHO_END)
endef

# COMPILEXX - Default macro to compile one C++ file
# Example: $(call COMPILEXX, in-file, out-file, flags)
#
# Depends on these settings defined in board-specific Make.defs file
# installed at $(TOPDIR)/Make.defs:
#
#   CXX - The command to invoke the C++ compiler
#   CXXFLAGS - Options to pass to the C++ compiler
#
# '<filename>.cxx_CXXFLAGS += <options>' may also be used, as an example, to
# change the options used with the single file <filename>.cxx.  The
# extension .cpp could also be used.  The same applies mutatis mutandis.

define COMPILEXX
	$(ECHO_BEGIN)"CXX: $1 "
	$(Q) $(CCACHE) $(CXX) -c $(CXXFLAGS) $3 $($(strip $1)_CXXFLAGS) $1 -o $2
	$(ECHO_END)
endef

# COMPILERUST - Default macro to compile one Rust file
# Example: $(call COMPILERUST, in-file, out-file)
#
# Depends on these settings defined in board-specific Make.defs file
# installed at $(TOPDIR)/Make.defs:
#
#   RUST - The command to invoke the Rust compiler
#   RUSTFLAGS - Options to pass to the Rust compiler
#
# '<filename>.rs_RUSTFLAGS += <options>' may also be used, as an example, to
# change the options used with the single file <filename>.rs. The same
# applies mutatis mutandis.

define COMPILERUST
	$(ECHO_BEGIN)"RUSTC: $1 "
	$(Q) $(RUSTC) --emit obj $(RUSTFLAGS) $($(strip $1)_RUSTFLAGS) $1 -o $2
	$(ECHO_END)
endef

# COMPILEZIG - Default macro to compile one Zig file
# Example: $(call COMPILEZIG, in-file, out-file)
#
# Depends on these settings defined in board-specific Make.defs file
# installed at $(TOPDIR)/Make.defs:
#
#   ZIG - The command to invoke the Zig compiler
#   ZIGFLAGS - Options to pass to the Zig compiler
#
# '<filename>.zig_ZIGFLAGS += <options>' may also be used, as an example, to
# change the options used with the single file <filename>.zig. The same
# applies mutatis mutandis.

define COMPILEZIG
	$(ECHO_BEGIN)"ZIG: $1 "
	$(Q) $(ZIG) build-obj $(ZIGFLAGS) $($(strip $1)_ZIGFLAGS) --name $(basename $2) $1
	$(ECHO_END)
endef

# ASSEMBLE - Default macro to assemble one assembly language file
# Example: $(call ASSEMBLE, in-file, out-file)
#
# NOTE that the most common toolchain, GCC, uses the compiler to assemble
# files because this has the advantage of running the C Pre-Processor against
# the assembly language files.  This is not possible with other toolchains;
# platforms using those other tools should define AS and over-ride this
# definition in order to use the assembler directly.
#
# Depends on these settings defined in board-specific Make.defs file
# installed at $(TOPDIR)/Make.defs:
#
#   CC - By default, the C compiler is used to compile assembly language
#        files
#   AFLAGS - Options to pass to the C+compiler
#
# '<filename>.s_AFLAGS += <options>' may also be used, as an example, to change
# the options used with the single file <filename>.s.  The extension .asm
# is used by some toolchains.  The same applies mutatis mutandis.

define ASSEMBLE
	$(ECHO_BEGIN)"AS: $1 "
	$(Q) $(CCACHE) $(CC) -c $(AFLAGS) $1 $($(strip $1)_AFLAGS) -o $2
	$(ECHO_END)
endef

# INSTALL_LIB - Install a library $1 into target $2
# Example: $(call INSTALL_LIB, libabc.a, $(TOPDIR)/staging/)

define INSTALL_LIB
	$(ECHO_BEGIN)"IN: $1 -> $2 "
	$(Q) install -m 0644 $1 $2
	$(ECHO_END)
endef

# ARCHIVE_ADD - Add a list of files to an archive
# Example: $(call ARCHIVE_ADD, archive-file, "file1 file2 file3 ...")
#
# Note: The fileN strings may not contain spaces or  characters that may be
# interpreted strangely by the shell
#
# Depends on these settings defined in board-specific Make.defs file
# installed at $(TOPDIR)/Make.defs:
#
#   AR - The command to invoke the archiver (includes any options)
#
# Depends on this settings defined in board-specific defconfig file installed
# at $(TOPDIR)/.config:
#
#   CONFIG_WINDOWS_NATIVE - Defined for a Windows native build

define ARCHIVE_ADD
	$(ECHO_BEGIN)"AR (add): ${shell basename $(1)} $(2) "
	$(Q) $(AR) $1 $2
	$(ECHO_END)
endef

# ARCHIVE - Same as above, but ensure the archive is
# created from scratch

define ARCHIVE
	$(Q) $(RM) $1
	$(Q) $(AR) $1  $2
endef

# PRELINK - Prelink a list of files
# This is useful when files were compiled with fvisibility=hidden.
# Any symbol which was not explicitly made global is invisible outside the
# prelinked file.
#
# Example: $(call PRELINK, prelink-file, "file1 file2 file3 ...")
#
# Note: The fileN strings may not contain spaces or  characters that may be
# interpreted strangely by the shell
#
# Depends on these settings defined in board-specific Make.defs file
# installed at $(TOPDIR)/Make.defs:
#
#   LD      - The command to invoke the linker (includes any options)
#   OBJCOPY - The command to invoke the object cop (includes any options)
#
# Depends on this settings defined in board-specific defconfig file installed
# at $(TOPDIR)/.config:
#
#   CONFIG_WINDOWS_NATIVE - Defined for a Windows native build

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define PRELINK
	@echo PRELINK: $1
	$(Q) $(LD) -Ur -o $1 $2 && $(OBJCOPY) --localize-hidden $1
endef
else
define PRELINK
	@echo "PRELINK: $1"
	$(Q) $(LD) -Ur -o $1 $2 && $(OBJCOPY) --localize-hidden $1
endef
endif

# POSTBUILD -- Perform post build operations
# Some architectures require the use of special tools and special handling
# AFTER building the NuttX binary.  Make.defs files for those architectures
# should override the following define with the correct operations for
# that platform

define POSTBUILD
endef

# DELFILE - Delete one file

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define NEWLINE


endef
define DELFILE
	$(foreach FILE, $(1), $(NEWLINE) $(Q) if exist $(FILE) (del /f /q  $(FILE)))
endef
else
define DELFILE
	$(Q) rm -f $1
endef
endif

# DELDIR - Delete one directory

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define DELDIR
	$(Q) if exist $1 (rmdir /q /s $1) $(NEWLINE)
endef
else
define DELDIR
	$(Q) rm -rf $1
endef
endif

# MOVEFILE - Move one file

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define MOVEFILE
	$(Q) if exist $1 (move /Y $1 $2)
endef
else
define MOVEFILE
	$(Q) mv -f $1 $2
endef
endif

# COPYFILE - Copy one file

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define COPYFILE
	$(Q) if exist $1 (copy /y /b $1 $2)
endef
else
define COPYFILE
	$(Q) cp -f $1 $2
endef
endif

# CATFILE - Cat a list of files
#
# USAGE: $(call CATFILE,dest,src1,src2,src3,...)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define CATFILE
	$(foreach FILE, $(2), $(NEWLINE) $(Q) type $(FILE) >> $1)
endef
else
define CATFILE
	$(Q) if [ -z "$(strip $(2))" ]; then echo '' > $(1); else cat $(2) > $1; fi
endef
endif

# RWILDCARD - Recursive wildcard used to get lists of files from directories
#
# USAGE:  FILELIST = $(call RWILDCARD,<dir>,<wildcard-filename)
#
# This is functionally equivalent to the following, but has the advantage in
# that it is portable
#
# FILELIST = ${shell find <dir> -name <wildcard-file>}

define RWILDCARD
  $(foreach d,$(wildcard $1/*),$(call RWILDCARD,$d,$2)$(filter $(subst *,%,$2),$d))
endef

# FINDSCRIPT - Find a given linker script. Prioritize the version from currently
#              configured board. If not provided, use the linker script from the
#              board common directory.
# Example: $(call FINDSCRIPT,script.ld)

define FINDSCRIPT
	$(if $(wildcard $(BOARD_DIR)$(DELIM)scripts$(DELIM)$(1)),$(BOARD_DIR)$(DELIM)scripts$(DELIM)$(1),$(BOARD_COMMON_DIR)$(DELIM)scripts$(DELIM)$(1))
endef

# CLEAN - Default clean target

ifeq ($(CONFIG_ARCH_COVERAGE),y)
	EXTRA = *.gcno *.gcda
endif

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define CLEAN
	$(Q) if exist *$(OBJEXT) (del /f /q *$(OBJEXT))
	$(Q) if exist *$(LIBEXT) (del /f /q *$(LIBEXT))
	$(Q) if exist *~ (del /f /q *~)
	$(Q) if exist (del /f /q  .*.swp)
	$(call DELFILE,$(subst /,\,$(OBJS)))
	$(Q) if exist $(BIN) (del /f /q  $(subst /,\,$(BIN)))
	$(Q) if exist $(EXTRA) (del /f /q  $(subst /,\,$(EXTRA)))
endef
else
define CLEAN
	$(Q) rm -f *$(OBJEXT) *$(LIBEXT) *~ .*.swp $(OBJS) $(BIN) $(EXTRA)
endef
endif

# TESTANDREPLACEFILE - Test if two files are different. If so replace the
#                      second with the first.  Otherwise, delete the first.
#
# USAGE:  $(call TESTANDREPLACEFILE, newfile, oldfile)
#
# args: $1 - newfile:  Temporary file to test
#       $2 - oldfile:  File to replace

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define TESTANDREPLACEFILE
	$(Q) move /Y $1 $2
endef
else
define TESTANDREPLACEFILE
	if [ -f $2 ]; then \
		if cmp -s $1 $2; then \
			rm -f $1; \
		else \
			mv $1 $2; \
		fi \
	else \
		mv $1 $2; \
	fi
endef
endif

# Invoke make

define MAKE_template
	+$(Q) $(MAKE) -C $(1) $(2) APPDIR="$(APPDIR)"

endef

define SDIR_template
$(1)_$(2):
	+$(Q) $(MAKE) -C $(1) $(2) APPDIR="$(APPDIR)"

endef

export DEFINE_PREFIX ?= $(subst X,,${shell $(DEFINE) "$(CC)" X 2> ${EMPTYFILE}})
export INCDIR_PREFIX ?= $(subst "X",,${shell $(INCDIR) "$(CC)" X 2> ${EMPTYFILE}})
export INCSYSDIR_PREFIX ?= $(subst "X",,${shell $(INCDIR) -s "$(CC)" X 2> ${EMPTYFILE}})

# ARCHxxx means the predefined setting(either toolchain, arch, or system specific)
ARCHDEFINES += ${DEFINE_PREFIX}__NuttX__
ifeq ($(CONFIG_NDEBUG),y)
  ARCHDEFINES += ${DEFINE_PREFIX}NDEBUG
endif

# The default C/C++ search path

ARCHINCLUDES += ${INCSYSDIR_PREFIX}$(TOPDIR)$(DELIM)include

ifeq ($(CONFIG_LIBCXX),y)
  ARCHXXINCLUDES += ${INCSYSDIR_PREFIX}$(TOPDIR)$(DELIM)include$(DELIM)libcxx
else ifeq ($(CONFIG_UCLIBCXX),y)
  ARCHXXINCLUDES += ${INCSYSDIR_PREFIX}$(TOPDIR)$(DELIM)include$(DELIM)uClibc++
else
  ARCHXXINCLUDES += ${INCSYSDIR_PREFIX}$(TOPDIR)$(DELIM)include$(DELIM)cxx
  ifeq ($(CONFIG_ETL),y)
    ARCHXXINCLUDES += ${INCSYSDIR_PREFIX}$(TOPDIR)$(DELIM)include$(DELIM)etl
  endif
endif
ARCHXXINCLUDES += ${INCSYSDIR_PREFIX}$(TOPDIR)$(DELIM)include

# Convert filepaths to their proper system format (i.e. Windows/Unix)

ifeq ($(CONFIG_CYGWIN_WINTOOL),y)
  CONVERT_PATH = $(foreach FILE,$1,${shell cygpath -w $(FILE)})
else
  CONVERT_PATH = $1
endif

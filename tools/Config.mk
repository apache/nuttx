############################################################################
# tools/Config.mk
# Global build rules and macros.
#
#   Copyright (C) 2011, 2013-2014, 2018-2019, 2020 Gregory Nutt. All rights
#     reserved.
#   Author: Richard Cochran
#           Gregory Nutt <gnutt@nuttx.org>
#
# This file (along with $(TOPDIR)/.config) must be included by every
# configuration-specific Make.defs file.
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

# Disable all built-in rules

.SUFFIXES:

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
export SHELL=cmd
endif

# Control build verbosity
#
#  V=1,2: Enable echo of commands
#  V=2:   Enable bug/verbose options in tools and scripts

ifeq ($(V),1)
export Q :=
else ifeq ($(V),2)
export Q :=
else
export Q := @
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

# Define HOSTCC on the make command line if it differs from these defaults
# Define HOSTCFLAGS with -g on the make command line to build debug versions

HOSTOS = ${shell uname -o 2>/dev/null || uname -s 2>/dev/null || echo "Other"}

ifeq ($(HOSTOS),MinGW)

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

ifeq ($(HOSTOS),Cygwin)
HOSTCFLAGS += -DHOST_CYGWIN=1
endif
endif

# Some defaults just to prohibit some bad behavior if for some reason they
# are not defined

ASMEXT ?= .S
OBJEXT ?= .o
LIBEXT ?= .a

ifeq ($(HOSTOS),Cygwin)
  EXEEXT ?= .exe
endif

ifeq ($(CONFIG_HOST_WINDOWS),y)
  HOSTEXEEXT ?= .exe
endif

# This define is passed as EXTRAFLAGS for kernel-mode builds.  It is also passed
# during PASS1 (but not PASS2) context and depend targets.

KDEFINE ?= ${shell $(DEFINE) "$(CC)" __KERNEL__}

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
else
  BOARD_DIR ?= $(TOPDIR)$(DELIM)boards$(DELIM)$(CONFIG_ARCH)$(DELIM)$(CONFIG_ARCH_CHIP)$(DELIM)$(CONFIG_ARCH_BOARD)
endif

BOARD_COMMON_DIR ?= $(wildcard $(BOARD_DIR)$(DELIM)..$(DELIM)common)
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
  DEFINE ?= "$(TOPDIR)\tools\define.bat"
  INCDIR ?= "$(TOPDIR)\tools\incdir.bat"
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
	@echo "CPP: $1->$2"
	$(Q) $(CPP) $(CPPFLAGS) $($(strip $1)_CPPFLAGS) $1 -o $2
endef

# COMPILE - Default macro to compile one C file
# Example: $(call COMPILE, in-file, out-file)
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
	@echo "CC: $1"
	$(Q) $(CC) -c $(CFLAGS) $($(strip $1)_CFLAGS) $1 -o $2
endef

# COMPILEXX - Default macro to compile one C++ file
# Example: $(call COMPILEXX, in-file, out-file)
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
	@echo "CXX: $1"
	$(Q) $(CXX) -c $(CXXFLAGS) $($(strip $1)_CXXFLAGS) $1 -o $2
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
	@echo "AS: $1"
	$(Q) $(CC) -c $(AFLAGS) $1 $($(strip $1)_AFLAGS) -o $2
endef

# INSTALL_LIB - Install a library $1 into target $2
# Example: $(call INSTALL_LIB, libabc.a, $(TOPDIR)/staging/)

define INSTALL_LIB
	@echo "IN: $1 -> $2"
	$(Q) install -m 0644 $1 $2
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
	@echo "AR (add): ${shell basename $(1)} $(2)"
	$(Q) $(AR) $1 $(2)
endef

# ARCHIVE - Same as above, but ensure the archive is
# created from scratch

define ARCHIVE
	@echo "AR (create): ${shell basename $(1)} $(2)"
	$(Q) $(RM) $1
	$(Q) $(AR) $1 $(2)
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
# AFTER building the NuttX binary.  Make.defs files for thos architectures
# should override the following define with the correct operations for
# that platform

define POSTBUILD
endef

# DELFILE - Delete one file

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define DELFILE
	$(Q) if exist $1 (del /f /q $1)
endef
else
define DELFILE
	$(Q) rm -f $1
endef
endif

# DELDIR - Delete one directory

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define DELDIR
	$(Q) if exist $1 (rmdir /q /s $1)
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
	$(Q) type $(2) > $1
endef
else
define CATFILE
	$(Q) cat $(2) > $1
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

# CLEAN - Default clean target

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define CLEAN
	$(Q) if exist *$(OBJEXT) (del /f /q *$(OBJEXT))
	$(Q) if exist *$(LIBEXT) (del /f /q *$(LIBEXT))
	$(Q) if exist *~ (del /f /q *~)
	$(Q) if exist (del /f /q  .*.swp)
	$(Q) if exist $(OBJS) (del /f /q $(OBJS))
	$(Q) if exist $(BIN) (del /f /q  $(BIN))
endef
else
define CLEAN
	$(Q) rm -f *$(OBJEXT) *$(LIBEXT) *~ .*.swp $(OBJS) $(BIN)
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
		if cmp $1 $2; then \
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

# The default C/C++ search path

ARCHINCLUDES += ${shell $(INCDIR) -s "$(CC)" $(TOPDIR)$(DELIM)include}

ifeq ($(CONFIG_LIBCXX),y)
  ARCHXXINCLUDES += ${shell $(INCDIR) -s "$(CC)" $(TOPDIR)$(DELIM)include$(DELIM)libcxx}
else ifeq ($(CONFIG_UCLIBCXX),y)
  ARCHXXINCLUDES += ${shell $(INCDIR) -s "$(CC)" $(TOPDIR)$(DELIM)include$(DELIM)uClibc++}
else
  ARCHXXINCLUDES += ${shell $(INCDIR) -s "$(CC)" $(TOPDIR)$(DELIM)include$(DELIM)cxx}
endif
ARCHXXINCLUDES += ${shell $(INCDIR) -s "$(CC)" $(TOPDIR)$(DELIM)include}

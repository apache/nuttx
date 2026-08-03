############################################################################
# tools/fdpic/nuttx-fdpic.mk
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

# nuttx-fdpic.mk -- build out-of-tree FDPIC modules for NuttX
#
# A module is an ELF shared object whose read-only segment the target maps
# straight out of flash and executes in place, and whose writable segment is
# copied to RAM once per running instance.  It links against nothing: libc
# and everything else are imported from the firmware's exported symbol table
# at load time.
#
# Usage -- a whole module is this:
#
#     MODULE = hello
#     SRCS   = hello.c
#     include /path/to/nuttx/tools/fdpic/nuttx-fdpic.mk
#
# C++ sources go in CXXSRCS instead of SRCS.
#
# Two toolchains are involved, and the split is the whole trick:
#
#   * the stock Arm bare-metal compiler does the compiling.  It emits
#     perfectly good FDPIC objects for both C and C++.
#   * arm-uclinuxfdpiceabi *binutils* does the linking, because
#     arm-none-eabi-ld is built with only the `armelf` emulation and cannot
#     produce an FDPIC object at all -- it silently marks the output
#     "UNIX - System V" and the loader refuses it.
#
# So the from-source dependency is binutils alone, which takes about a
# minute to build.  No FDPIC GCC is needed.  See build-binutils.sh beside
# this file, and Documentation/components/fdpic.rst.
#
# Required:
#   NUTTX_DIR        a configured, built NuttX tree (headers + export list)
#
# Optional:
#   ARM_TOOLCHAIN    prefix of the bare-metal compiler; default arm-none-eabi
#   FDPIC_TOOLCHAIN  prefix of the FDPIC binutils; default
#                    arm-uclinuxfdpiceabi
#   CPU              default cortex-m33
#   ENTRY            entry symbol; default main, use 0 for a library
#   OPT              default -Os
#   LIBS             shared libraries to link against
#   BINDNOW          '-z now' by default; set empty to leave imported
#                    descriptors in the lazy binding table (DT_JMPREL)
#   EXTRA_CFLAGS EXTRA_CXXFLAGS EXTRA_LDFLAGS
#
# EXTRA_LDFLAGS is passed straight to ld, not through a compiler driver, so
# it takes bare linker flags: `-soname libfoo.so`, not `-Wl,-soname,libfoo.so`.

MODULE  ?= module
SRCS    ?=
CXXSRCS ?=
CPU     ?= cortex-m33
ENTRY   ?= main
OPT     ?= -Os

ARM_TOOLCHAIN   ?= arm-none-eabi
FDPIC_TOOLCHAIN ?= arm-uclinuxfdpiceabi

FDPIC_DIR := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))

ifeq ($(NUTTX_DIR),)
  $(error Set NUTTX_DIR to a configured, built NuttX tree)
endif

# make has built-in defaults for CC and CXX, so ?= never fires for them and
# the host compiler silently gets the job.  Test the origin instead.

ifeq ($(origin CC),default)
  CC := $(ARM_TOOLCHAIN)-gcc
endif

ifeq ($(origin CXX),default)
  CXX := $(ARM_TOOLCHAIN)-g++
endif

LD      := $(FDPIC_TOOLCHAIN)-ld
READELF := $(FDPIC_TOOLCHAIN)-readelf

# Common compile flags.
#
# -mfdpic is stated rather than assumed, so a mis-set toolchain fails loudly
# instead of quietly producing a plain ELF the loader will refuse.
#
# -fPIC is not optional and not implied.  -mfdpic alone does not turn on PIC
# under the bare-metal compiler, and without it the link emits TEXTREL --
# text relocations -- which cannot work when the text is executed in place
# out of read-only flash.
#
# -fno-builtin keeps GCC from open-coding calls into libc routines the
# module is supposed to import from the firmware.
#
# __STDC_NO_ATOMICS__ steers NuttX's <nuttx/atomic.h> away from the branch
# that includes <stdatomic.h> and then redefines its macros.  The effect is
# that a module using C11 atomics gets NuttX's implementation -- the same one
# the firmware uses -- rather than the compiler's.

MODCOMMON = -mcpu=$(CPU) -mthumb -mfdpic -fPIC $(OPT) \
            -fno-builtin -Wall -Wa,--noexecstack \
            -D__STDC_NO_ATOMICS__ -D__NuttX__

MODCFLAGS = $(MODCOMMON) -I$(NUTTX_DIR)/include $(EXTRA_CFLAGS)

# C++ adds three flags, none of them optional.
#
# -fno-use-cxa-atexit, because the default registers each static object's
# destructor with __cxa_atexit(dtor, obj, &__dso_handle), and __dso_handle
# comes from crtbegin, which a module does not link.  The link fails
# outright with "hidden symbol `__dso_handle' isn't defined".  Turning it off
# also puts the destructors in .fini_array, which is what the loader walks on
# unload -- so the flag that makes the link work is also the flag that makes
# destructors run.
#
# -fno-exceptions -fno-rtti, because both need libsupc++, which a module
# linking against nothing cannot reach.

MODCXXFLAGS = $(MODCOMMON) \
              -fno-exceptions -fno-rtti -fno-use-cxa-atexit \
              -I$(NUTTX_DIR)/include/cxx -I$(NUTTX_DIR)/include \
              $(EXTRA_CXXFLAGS)

# Link flags, passed straight to ld.
#
# -shared is load bearing and easy to get wrong.  It is what preserves
# R_ARM_FUNCDESC_VALUE relocations for imported symbols.  Linking as a PIE
# with --unresolved-symbols=ignore-all also appears to work, but silently
# degrades every import to R_ARM_NONE, and the module then branches to zero
# on its first call out.
#
# The emulation has to be named because this ld supports four.
#
# -z now keeps imported function descriptors in DT_REL rather than the lazy
# binding table DT_JMPREL.  The loader binds both, so this is a default rather
# than a requirement: it keeps built modules on the layout that has had the
# most hardware exposure, and leaves the lazymod fixture, which empties
# BINDNOW, a distinct case rather than what everything does.

BINDNOW ?= -z now

MODLDFLAGS = -m armelf_linux_fdpiceabi -shared $(BINDNOW) -e $(ENTRY) \
             $(EXTRA_LDFLAGS)

OBJS    := $(SRCS:.c=.o) $(CXXSRCS:.cpp=.o)
TARGET  := $(MODULE).fdpic
EXPORTS := .nuttx-exports

.PHONY: all clean verify exports

all: verify

$(EXPORTS): $(NUTTX_DIR)/libs/libc/exec_symtab.c
	@$(FDPIC_DIR)/nuttx-exports.sh $(NUTTX_DIR) > $@

exports: $(EXPORTS)
	@echo "$(shell wc -l < $(EXPORTS)) symbols exported by the firmware"

%.o: %.c
	@echo "  CC   $<"
	@$(CC) $(MODCFLAGS) -c $< -o $@

%.o: %.cpp
	@echo "  CXX  $<"
	@$(CXX) $(MODCXXFLAGS) -c $< -o $@

$(TARGET): $(OBJS)
	@echo "  LD   $@"
	@$(LD) $(MODLDFLAGS) -o $@ $(OBJS) $(LIBS)

# Verification is part of the default build on purpose.  A module that
# imports a symbol the firmware does not export links perfectly happily and
# only fails once it is on the target, as a bare -ENOENT with no indication
# of which symbol was at fault.

verify: $(TARGET) $(EXPORTS)
	@READELF=$(READELF) $(FDPIC_DIR)/fdpic-verify.sh \
	    $(TARGET) $(EXPORTS) $(LIBS)

clean:
	@rm -f $(OBJS) $(TARGET) $(MODULE).so $(EXPORTS)

# ##############################################################################
# arch/sim/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

include_directories(include)
add_subdirectory(src)

# configure host binary ######################################################

target_include_directories(nuttx PRIVATE ${CONFIG_ARCH_CHIP})

# This is part of the top-level export target TODO: how to deal with in CMake?

# export_startup: board/libboard$(LIBEXT) up_head.o $(HOSTOBJS) nuttx-names.dat
# cp up_head.o $(HOSTOBJS) ${EXPORT_DIR}/startup cp nuttx-names.dat
# ${EXPORT_DIR}/libs echo main NXmain >> ${EXPORT_DIR}/libs/nuttx-names.dat

# Loadable module definitions TODO: implement modules with CMake

# -fno-pic to avoid GOT relocations
set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_ELF_MODULE_COMPILE_OPTIONS -fno-pic)
set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_ELF_MODULE_LINK_OPTIONS -r -e module_initialize -T
           ${NUTTX_DIR}/libs/libc/modlib/gnu-elf.ld)

if(CONFIG_LIBC_ARCH_ELF_64BIT)
  # For amd64: It seems macOS/x86_64 loads the program text around
  # 00000001_xxxxxxxx. The gcc default (-mcmodel=small) would produce
  # out-of-range 32-bit relocations. Even on Linux, NuttX modules are loaded
  # into the NuttX heap, which can be out of range with -mcmodel=small.
  set_property(
    TARGET nuttx
    APPEND
    PROPERTY NUTTX_ELF_MODULE_COMPILE_OPTIONS -mcmodel=large)
endif()

# On Linux, we (ab)use the host compiler to compile binaries for NuttX.
# Explicitly disable features which might be default on the host while not
# available on NuttX.
set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_ELF_MODULE_COMPILE_OPTIONS -fno-stack-protector)

# TODO: move to toolchain file NuttX modules are ELF binaries. Non-ELF platforms
# like macOS need to use a separate ELF toolchain. ifeq ($(CONFIG_HOST_MACOS),y)
# # eg. brew install x86_64-elf-gcc MODULECC = x86_64-elf-gcc MODULELD =
# x86_64-elf-ld MODULESTRIP = x86_64-elf-strip --strip-unneeded endif

# ELF module definitions

# -fno-pic to avoid GOT relocations
set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_ELF_APP_COMPILE_OPTIONS -fno-pic)
set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_ELF_APP_LINK_OPTIONS -r -e main
           -T${BOARD_PATH}/scripts/gnu-elf.ld)

# TODO: move to toolchain file
if(X86_64 AND CONFIG_SIM_M32)
  set_property(
    TARGET nuttx
    APPEND
    PROPERTY NUTTX_ELF_MODULE_LINK_OPTIONS -m32)
  set_property(
    TARGET nuttx
    APPEND
    PROPERTY NUTTX_ELF_APP_LINK_OPTIONS -m32)
endif()

# ##############################################################################
# arch/tricore/src/cmake/Toolchain.cmake
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

# Toolchain

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

set(ARCH_SUBDIR)

if(CONFIG_ARCH_TC3XX) # TC3XX
  set(ARCH_SUBDIR tc3xx)
else()
  set(ARCH_SUBDIR tc3xx)
endif()

include(${ARCH_SUBDIR})

set(CMAKE_ASM_COMPILER cctc)
set(CMAKE_C_COMPILER ${CMAKE_ASM_COMPILER})
set(CMAKE_CXX_COMPILER cctc)
set(CMAKE_STRIP strip --strip-unneeded)
set(CMAKE_OBJCOPY echo)
set(CMAKE_OBJDUMP elfdump)

set(CMAKE_LINKER cctc)
set(CMAKE_LD cctc)
set(CMAKE_AR artc -r)
set(CMAKE_NM nm)
set(CMAKE_RANLIB ranlib)

# override the ARCHIVE command

set(CMAKE_C_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_ASM_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")

# Architecture flags

add_compile_options(--lsl-core=vtc)
add_compile_options(--iso=99)
add_compile_options(--language=+gcc,+volatile,-strings,-kanji)

if(CONFIG_DEBUG_CUSTOMOPT)
  add_compile_options(${CONFIG_DEBUG_OPTLEVEL})
elseif(CONFIG_DEBUG_FULLOPT)
  add_compile_options(-Os)
endif()

# pragma align <4> (default: 0)

add_compile_options(--align=4)

# Always use 32-bit integers for enumeration

add_compile_options(--integer-enumeration)

# tradeoff between speed (-t0) and size (-t4) (default: 4)

add_compile_options(--tradeoff=2)

# enable symbolic debug information

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(--debug-info=default)
  add_compile_options(--keep-temporary-files)
  add_link_options(-g)
endif()

# merge source code with assembly output

add_compile_options(--source)

# generate alignment depending on assume_if hints

add_compile_options(--branch-target-align)

# cmake-format: off
# Since nuttx uses too many of GNU extensions in the implementation of
# FPU-related library functions, which is not supported in tasking, so currently
# we cannot use FPU-related configurations to manage it.
#
# Just set fp-model to Double Precision:
# --fp-model[=<flag>,...]         floating-point model (default: cFlnrSTz)
#   0                               alias for --fp-model=CFLNRStZ (strict)
#   1                               alias for --fp-model=cFLNRSTZ (precise)
#   2                               alias for --fp-model=cFlnrSTz (fast-dp)
#   3                               alias for --fp-model=cflnrSTz (fast-sp)
# cmake-format: on

add_compile_options(--fp-model=2)
add_link_options(--fp-model=2)
add_link_options(-lfp_fpu)

add_link_options(--hex-format=s -Wl-OtxYcL -Wl-mcrfiklsmnoduq)
add_link_options(-lrt)

# cmake-format: off
# ctc W500: ["stdio/lib_libvsprintf.c" 884/29] expression without effect
# ctc W507: ["mm_heap/mm_malloc.c" 238/64] variable "nodesize" is possibly uninitialized
# ctc W508: ["misc/lib_impure.c" 1/1] empty source file
# ctc W525: ["getopt.c" 678/3] discarded 'const' qualifier at assignment: conversion from char const * to char *
# ctc W527: ["stdlib/lib_strtold.c" 565/23] constant of type "double" saturated
# ctc W526: ["include/sys/epoll.h" 87/5] enumeration constant shall be representable as 'int'
# ctc W529: ["wchar/lib_mbrtowc.c" 88/35] overflow in constant expression of type "unsigned long int"
# ctc W544: ["wqueue/kwork_thread.c" 210/32] unreachable code
# ctc W549: ["unistd/lib_getopt_common.c" 544/15] condition is always true
# ctc W553: ["vfs/fs_fcntl.c" 231/7] no 'break' or comment before case label
# ctc W557: ["common/tricore_main.c" 58/11] possible infinite loop
# ctc W560: ["tmpfs/fs_tmpfs.c" 232/25] possible truncation at implicit conversion to type "unsigned short int"
# ctc W562: ["mm_heap/mm_memalign.c" 70/20] unary minus applied to unsigned value
# ctc W558: ["include/nuttx/power/regulator.h" 224/36] struct/union/enum definition in parameter declaration
# ctc W587: ["stdlib/lib_strtold.c" 571/23] underflow on constant of type "double"
# ctc W588: ["misc/lib_glob.c" 150/13] dead assignment to "i" eliminated
# ctc W589: ["inode/fs_inodesearch.c" 72/8] pointer assumed to be nonzero - test removed
# cmake-format: on

set(TASKING_WARNINGS
    500,507,508,525,526,527,529,544,549,553,560,562,557,558,587,588,589)

add_compile_options(--pass-c=--no-warnings=${TASKING_WARNINGS})

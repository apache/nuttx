# ##############################################################################
# arch/tricore/src/cmake/ToolchainTasking.cmake
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

if(CONFIG_ARCH_CHIP_TC3XX)
  set(ARCH_SUBDIR tc3xx)
elseif(CONFIG_ARCH_CHIP_TC4XX)
  set(ARCH_SUBDIR tc4xx)
endif()

include(chip)

set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}cctc)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}cctc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}cctc)
set(CMAKE_STRIP strip --strip-unneeded)
set(CMAKE_OBJCOPY echo)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}elfdump)

set(CMAKE_LINKER ${TOOLCHAIN_PREFIX}ltc)
set(CMAKE_LD ${TOOLCHAIN_PREFIX}ltc)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}artc -r)
set(CMAKE_NM nm)
set(CMAKE_RANLIB ranlib)

set(ASMEXT
    asm
    CACHE INTERNAL "")

set(CMAKE_C_COMPILE_OBJECT
    "<CMAKE_C_COMPILER> <DEFINES> <INCLUDES> <FLAGS> --create <SOURCE> -o <OBJECT>"
)
set(CMAKE_CXX_COMPILE_OBJECT
    "<CMAKE_CXX_COMPILER> <DEFINES> <INCLUDES> <FLAGS> --create <SOURCE> -o <OBJECT>"
)

if(CMAKE_GENERATOR MATCHES "Ninja")
  set(CMAKE_C_RESPONSE_FILE_FLAG "-f ${CMAKE_BINARY_DIR}/")
  set(CMAKE_CXX_RESPONSE_FILE_FLAG "-f ${CMAKE_BINARY_DIR}/")
  set(CMAKE_ASM_RESPONSE_FILE_FLAG "-f ${CMAKE_BINARY_DIR}/")
  set(CMAKE_C_RESPONSE_FILE_LINK_FLAG "-f ${CMAKE_BINARY_DIR}/")
  set(CMAKE_CXX_RESPONSE_FILE_LINK_FLAG "-f ${CMAKE_BINARY_DIR}/")
  set(CMAKE_ASM_RESPONSE_FILE_LINK_FLAG "-f ${CMAKE_BINARY_DIR}/")
endif()

# override the ARCHIVE command

set(CMAKE_C_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_ASM_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")

# override the LINK command

add_link_options(-I${CMAKE_BINARY_DIR}/include)
set(CMAKE_C_LINK_EXECUTABLE
    "<CMAKE_LINKER> <FLAGS> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>"
)
set(CMAKE_CXX_LINK_EXECUTABLE
    "<CMAKE_LINKER> <FLAGS> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> <OBJECTS> -o <TARGET> <LINK_LIBRARIES>"
)

# Architecture flags

add_compile_options(--lsl-core=vtc)
add_link_options(-v)
if(CONFIG_ARCH_CHIP_TC3XX)
  add_compile_options($<$<COMPILE_LANGUAGE:C>:--iso=11>)
elseif(CONFIG_ARCH_CHIP_TC4XX)
  add_compile_options($<$<COMPILE_LANGUAGE:C>:--iso=17>)
endif()
add_compile_options(
  $<$<COMPILE_LANGUAGE:C>:--language=+gcc,+volatile,-strings,-kanji>)
add_compile_options($<$<COMPILE_LANGUAGE:C>:--no-stdinc>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:--no-stdinc>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:--c++=14>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:--g++>)
add_compile_options(
  $<$<COMPILE_LANGUAGE:CXX>:--language=+gcc,+volatile,-strings,-kanji>)
add_compile_options(--pass-c++=-D__CTC__)

function(get_smartcode_ctc_root OUT_VAR)
  find_program(_CCTC_PATH cctc)
  if(NOT _CCTC_PATH)
    message(FATAL_ERROR "get_smartcode_ctc_root: cannot find 'cctc' in PATH")
  endif()
  get_filename_component(_BIN_DIR "${_CCTC_PATH}" DIRECTORY)
  get_filename_component(_ROOT_DIR "${_BIN_DIR}" DIRECTORY)
  set(${OUT_VAR}
      "${_ROOT_DIR}"
      PARENT_SCOPE)
endfunction()

get_smartcode_ctc_root(SMARTCODE_CTC_ROOT)
add_compile_options(-I${SMARTCODE_CTC_ROOT}/include)

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

# mergering of sections

add_compile_options(--pass-assembler=--concatenate-sections)

# Debug link map

if(CONFIG_DEBUG_LINK_MAP)
  add_link_options(--map-file=nuttx.map)
endif()

# enable symbolic debug information

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(--debug-info=default)
  add_compile_options(--keep-temporary-files)
  add_compile_options(${CONFIG_DEBUG_SYMBOLS_LEVEL})
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
add_link_options(-lc_fpu)
add_link_options(-lfp_fpu)

add_link_options(--core=mpe:vtc)
add_link_options(--hex-format=s -OtxYcL -mcrfiklsmnoduq)
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
# cptc W0068: ["include/libcxx/string" 878] integer conversion resulted in a change of sign
# cptc W0161: ["SmartCode/ctc/include/math.h" 390] unrecognized #pragma
# cptc W0940: ["libcxx/src/include/to_chars_floating_point.h" 1073] missing return statement at end of non-void function "
# cptc W1097: ["include/libcxx/vector" 1347] unknown attribute "__visibility__"
# cptc W1105: ["libcxx/src/support/runtime/exception_pointer_unimplemented.ipp" 28] #warning directive: exception_ptr not yet implemented
# cptc W1315: ["include/libcxx/new" 183] function declared with "noreturn" does return (throw triggered)
# cptc W1858: ["libcxx/__bit/bit_cast.h" 34] attribute "__always_inline__" does not apply here
# cptc W2213: ["include/libcxx/string" 4110] GNU attributes on a template redeclaration have no effect (the attributes of the original declaration at line 656 apply instead)
# cptc W2496: ["include/libcxx/vector" 1429] the "always_inline" attribute is ignored on non-inline functions
# cptc W2529: ["libcxx/__chrono/duration.h" 559] a user-provided literal suffix must begin with "_"
# cptc W2964: ["libcxx/src/include/to_chars_floating_point.h" 994] constexpr if statements are a C++17 feature
# cptc W3085: ["include/libcxx/__charconv/tables.h" 26] inline variables are a C++17 feature
# cmake-format: on

set(TASKING_WARNINGS
    500,507,508,525,526,527,529,544,549,553,560,562,557,558,587,588,589)
set(TASKING_CXX_WARNINGS
    68,161,940,1097,1105,1315,1858,2213,2496,2529,2964,3085)

add_compile_options(--pass-c=--no-warnings=${TASKING_WARNINGS})
add_compile_options(--pass-c++=--no-warnings=${TASKING_CXX_WARNINGS})

if(NOT CONFIG_CXX_EXCEPTION)
  add_compile_options(--pass-c++=--no-exceptions)
endif()

if(CONFIG_CXX_RTTI)
  add_compile_options(--pass-c++=--rtti)
endif()

set(NUTTX_FIND_TOOLCHAIN_LIB_DEFINED true)
function(nuttx_find_toolchain_lib)
  get_filename_component(TASKING_ROOT_PATH "${CMAKE_C_COMPILER}" DIRECTORY)
  string(REGEX REPLACE "^/([a-zA-Z])/(.*)$" "\\1:/\\2" TASKING_ROOT_TEMP_PATH
                       "${TASKING_ROOT_PATH}")
  string(STRIP "${TASKING_ROOT_TEMP_PATH}" TASKING_ROOT_PATH)
  if(ARGN)
    if(CONFIG_ARCH_CHIP_TC3XX)
      nuttx_add_extra_library(${TASKING_ROOT_PATH}/../lib/tc162/${ARGN})
    elseif(CONFIG_ARCH_CHIP_TC4XX)
      nuttx_add_extra_library(${TASKING_ROOT_PATH}/../lib/tc18/${ARGN})
    endif()
  endif()
endfunction()

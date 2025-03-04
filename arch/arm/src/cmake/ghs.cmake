# ##############################################################################
# arch/arm/src/cmake/ghs.cmake
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

set(CMAKE_ASM_COMPILER ccarm)
set(CMAKE_C_COMPILER ccarm)
set(CMAKE_CXX_COMPILER ccarm)
set(CMAKE_STRIP gstrip)
set(CMAKE_OBJCOPY objcopy)
set(CMAKE_OBJDUMP gdump)
set(CMAKE_LINKER cxarm)
set(CMAKE_LD cxarm)
set(CMAKE_AR cxarm)
set(CMAKE_GMEMFILE gmemfile)
set(CMAKE_NM gnm)
set(CMAKE_RANLIB echo)
set(CMAKE_PREPROCESSOR ccarm -E -P)

# override the ARCHIVE command

set(CMAKE_ARCHIVE_COMMAND
    "<CMAKE_AR> <LINK_FLAGS> -archive <OBJECTS> -o <TARGET>")
set(CMAKE_C_ARCHIVE_CREATE ${CMAKE_ARCHIVE_COMMAND})
set(CMAKE_CXX_ARCHIVE_CREATE ${CMAKE_ARCHIVE_COMMAND})
set(CMAKE_ASM_ARCHIVE_CREATE ${CMAKE_ARCHIVE_COMMAND})

set(CMAKE_C_ARCHIVE_APPEND ${CMAKE_ARCHIVE_COMMAND})
set(CMAKE_CXX_ARCHIVE_APPEND ${CMAKE_ARCHIVE_COMMAND})
set(CMAKE_ASM_ARCHIVE_APPEND ${CMAKE_ARCHIVE_COMMAND})

set(NO_LTO "-Onolink")

if(CONFIG_ENDIAN_BIG)
  add_compile_options(-mbig-endian)
endif()

# Architecture flags

add_link_options(-entry=__start)
add_compile_options(--no_commons -Wall -Wshadow -Wundef -nostdlib)

if(CONFIG_DEBUG_CUSTOMOPT)
  add_compile_options(${CONFIG_DEBUG_OPTLEVEL})
elseif(CONFIG_DEBUG_FULLOPT)
  add_compile_options(-Osize)
endif()

if(NOT CONFIG_DEBUG_NOOPT)
  add_compile_options(-fno-strict-aliasing)
endif()

if(CONFIG_FRAME_POINTER)
  add_compile_options(-fno-omit-frame-pointer -fno-optimize-sibling-calls)
else()
  add_compile_options(-noga)
endif()

if(CONFIG_STACK_CANARIES)
  add_compile_options(-fstack-protector-all)
endif()

if(CONFIG_STACK_USAGE)
  add_compile_options(-fstack-usage)
endif()
if(CONFIG_STACK_USAGE_WARNING AND NOT "${CONFIG_STACK_USAGE_WARNING}" STREQUAL
                                  "0")
  add_compile_options(-Wstack-usage=${CONFIG_STACK_USAGE_WARNING})
endif()

if(CONFIG_COVERAGE_ALL)
  add_compile_options(-fprofile-arcs -ftest-coverage -fno-inline)
endif()

if(CONFIG_PROFILE_ALL)
  add_compile_options(-pg)
endif()

if(CONFIG_MM_UBSAN_ALL)
  add_compile_options(${CONFIG_MM_UBSAN_OPTION})
endif()

if(CONFIG_MM_UBSAN_TRAP_ON_ERROR)
  add_compile_options(-fsanitize-undefined-trap-on-error)
endif()

if(CONFIG_MM_KASAN_INSTRUMENT_ALL)
  add_compile_options(-fsanitize=kernel-address)
endif()

if(CONFIG_MM_KASAN_GLOBAL)
  add_compile_options(--param=asan-globals=1)
endif()

if(CONFIG_MM_KASAN_DISABLE_READS_CHECK)
  add_compile_options(--param=asan-instrument-reads=0)
endif()

if(CONFIG_MM_KASAN_DISABLE_WRITES_CHECK)
  add_compile_options(--param=asan-instrument-writes=0)
endif()

# Instrumentation options

if(CONFIG_ARCH_INSTRUMENT_ALL)
  add_compile_options(-finstrument-functions)
endif()

if(CONFIG_UNWINDER_ARM)
  add_compile_options(-funwind-tables -fasynchronous-unwind-tables)
endif()

# Link Time Optimization

if(CONFIG_LTO_THIN)
  add_compile_options(-Olink -Ogeneral)
elseif(CONFIG_LTO_FULL)
  add_compile_options(-Olink -Osize)
elseif(CONFIG_LTO_FAT)
  add_compile_options(-Olink -Ospeed)
endif()

if(CONFIG_ARM_THUMB)
  add_compile_options(-thumb)
endif()

# Debug --whole-archive

if(CONFIG_DEBUG_LINK_WHOLE_ARCHIVE)
  add_link_options(-Wl,--whole-archive)
endif()

# Debug link map

if(CONFIG_DEBUG_LINK_MAP)
  add_link_options(-map=nuttx.map)
endif()

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(-G -gdwarf-2)
endif()

add_compile_options(
  $<$<COMPILE_LANGUAGE:ASM>:-preprocess_assembly_files>
  $<$<COMPILE_LANGUAGE:ASM>:--gnu_asm>
  $<$<COMPILE_LANGUAGE:C>:-gcc>
  $<$<COMPILE_LANGUAGE:C>:-gnu99>
  $<$<COMPILE_LANGUAGE:C>:-preprocess_assembly_files>
  $<$<COMPILE_LANGUAGE:C>:--diag_suppress=68,111,174,222,236,257,826,1143,1721>
  $<$<COMPILE_LANGUAGE:CXX>:--gnu_asm>
  $<$<COMPILE_LANGUAGE:CXX>:--diag_suppress=826>)

if(CONFIG_CXX_STANDARD)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=${CONFIG_CXX_STANDARD}>)
endif()

if(NOT CONFIG_CXX_EXCEPTION)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
                      $<$<COMPILE_LANGUAGE:CXX>:-fcheck-new>)
endif()

if(CONFIG_CXX_STANDARD)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:--${CONFIG_CXX_STANDARD}>)
endif()

if(NOT CONFIG_CXX_EXCEPTION)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:--no_exceptions>
                      $<$<COMPILE_LANGUAGE:CXX>:-check=alloc>)
endif()

if(NOT CONFIG_CXX_RTTI)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>)
endif()

set(PREPROCESS ${CMAKE_C_COMPILER} ${CMAKE_C_FLAG_ARGS} -E -P)

# override nuttx_generate_preprocess_target

set(NUTTX_TOOLCHAIN_PREPROCESS_DEFINED true)

function(nuttx_generate_preprocess_target)

  # parse arguments into variables

  nuttx_parse_function_args(
    FUNC
    nuttx_generate_preprocess_target
    ONE_VALUE
    SOURCE_FILE
    TARGET_FILE
    MULTI_VALUE
    DEPENDS
    REQUIRED
    SOURCE_FILE
    TARGET_FILE
    ARGN
    ${ARGN})

  add_custom_command(
    OUTPUT ${TARGET_FILE}
    COMMAND ${PREPROCESS} -I${CMAKE_BINARY_DIR}/include -filetype.cpp
            ${SOURCE_FILE} -o ${TARGET_FILE}
    DEPENDS ${SOURCE_FILE} ${DEPENDS})

endfunction()

# override nuttx_find_toolchain_lib

set(NUTTX_FIND_TOOLCHAIN_LIB_DEFINED true)

function(nuttx_find_toolchain_lib)
  execute_process(
    COMMAND bash -c "which ${CMAKE_C_COMPILER} | awk -F '/[^/]*$' '{print $1}'"
    OUTPUT_VARIABLE GHS_ROOT_PATH)
  string(STRIP "${GHS_ROOT_PATH}" GHS_ROOT_PATH)
  if(NOT ARGN)
    nuttx_add_extra_library(${GHS_ROOT_PATH}/lib/thumb2/libarch.a)
    if(CONFIG_ARCH_FPU)
      if(CONFIG_ARM_FPU_ABI_SOFT)
        nuttx_add_extra_library(${GHS_ROOT_PATH}/lib/thumb2/libind_sf.a)
        nuttx_add_extra_library(${GHS_ROOT_PATH}/lib/thumb2/libmath_sf.a)
      elseif(CONFIG_ARCH_DPFPU)
        nuttx_add_extra_library(${GHS_ROOT_PATH}/lib/thumb2/libind_fp.a)
        nuttx_add_extra_library(${GHS_ROOT_PATH}/lib/thumb2/libmath_fp.a)
      else()
        nuttx_add_extra_library(${GHS_ROOT_PATH}/lib/thumb2/libind_sd.a)
        nuttx_add_extra_library(${GHS_ROOT_PATH}/lib/thumb2/libmath_sd.a)
      endif()
    else()
      nuttx_add_extra_library(${GHS_ROOT_PATH}/lib/thumb2/libind_sf.a)
      nuttx_add_extra_library(${GHS_ROOT_PATH}/lib/thumb2/libmath_sf.a)
    endif()
  endif()
endfunction()

# disable nuttx cmake link group otption
set(DISABLE_LINK_GROUP true)

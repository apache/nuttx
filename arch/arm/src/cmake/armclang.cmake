# ##############################################################################
# arch/arm/src/cmake/armclang.cmake
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

set(CMAKE_ASM_COMPILER armclang)
set(CMAKE_C_COMPILER armclang)
set(CMAKE_CXX_COMPILER armclang)
set(CMAKE_PREPROCESSOR armclang -E -P -x c)
set(CMAKE_STRIP llvm-strip --strip-unneeded)
set(CMAKE_OBJCOPY llvm-objcopy)
set(CMAKE_OBJDUMP llvm-objdump)
set(CMAKE_LINKER armlink)
set(CMAKE_LD armlink)
set(CMAKE_AR armar -rcs)
set(CMAKE_NM llvm-nm)
set(CMAKE_RANLIB llvm-ranlib)

# Since the no_builtin attribute is not fully supported on Clang disable the
# built-in functions, refer: https://github.com/apache/nuttx/pull/5971

add_compile_options(-fno-builtin --target=arm-arm-none-eabi)

# Suppress license warning

add_compile_options(-Wno-license-management)
add_link_options(-Wl,--diag_suppress=9931)
# Input sections are specified even though there will be no such sections found
# in the libraries linked. Warning: L6314W: No section matches pattern *(xxx).

add_link_options(-Wl,--diag_suppress=6314)

# Allow Empty Execution region declared on scatter Warning: L6312W: Empty
# Execution region description for region xxx

add_link_options(-Wl,--diag_suppress=6312)

# Match pattern for an unused section that is being removed. Warning: L6329W:
# Pattern xxx only matches removed unused sections.

add_link_options(-Wl,--diag_suppress=6329)

# override the ARCHIVE command

set(CMAKE_ARCHIVE_COMMAND "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_RANLIB_COMMAND "<CMAKE_RANLIB> <TARGET>")
set(CMAKE_C_ARCHIVE_CREATE ${CMAKE_ARCHIVE_COMMAND})
set(CMAKE_CXX_ARCHIVE_CREATE ${CMAKE_ARCHIVE_COMMAND})
set(CMAKE_ASM_ARCHIVE_CREATE ${CMAKE_ARCHIVE_COMMAND})

set(CMAKE_C_ARCHIVE_APPEND ${CMAKE_ARCHIVE_COMMAND})
set(CMAKE_CXX_ARCHIVE_APPEND ${CMAKE_ARCHIVE_COMMAND})
set(CMAKE_ASM_ARCHIVE_APPEND ${CMAKE_ARCHIVE_COMMAND})

set(CMAKE_C_ARCHIVE_FINISH ${CMAKE_RANLIB_COMMAND})
set(CMAKE_CXX_ARCHIVE_FINISH ${CMAKE_RANLIB_COMMAND})
set(CMAKE_ASM_ARCHIVE_FINISH ${CMAKE_RANLIB_COMMAND})

set(NO_LTO "-fno-lto")

if(CONFIG_ENDIAN_BIG)
  add_compile_options(-mbig-endian)
endif()

# Architecture flags

add_link_options(-Wl,--entry=__start)
add_link_options(-nostdlib)
add_compile_options(-fno-common -Wall -Wshadow -Wundef -nostdlib)

if(CONFIG_DEBUG_CUSTOMOPT)
  add_compile_options(${CONFIG_DEBUG_OPTLEVEL})
elseif(CONFIG_DEBUG_FULLOPT)
  add_compile_options(-Os)
endif()

if(NOT CONFIG_DEBUG_NOOPT)
  add_compile_options(-fno-strict-aliasing)
endif()

if(CONFIG_FRAME_POINTER)
  add_compile_options(-fno-omit-frame-pointer -fno-optimize-sibling-calls)
else()
  add_compile_options(-fomit-frame-pointer)
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
  add_compile_options(-fprofile-instr-generate -fcoverage-mapping)
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

if(CONFIG_MM_KASAN_ALL)
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
  add_compile_options(-flto=thin)
elseif(CONFIG_LTO_FULL)
  add_compile_options(-flto)

elseif(CONFIG_LTO_FAT)
  add_compile_options(-flto -ffat-lto-objects)
endif()

# The arm clang toolchain requires to pass the linker option will gcc tool chain
# can automatically perform lto at linking time if it found any object files are
# compiled with flto

if(NOT CONFIG_LTO_NONE)
  add_link_options(-Wl,--lto)
endif()

if(CONFIG_ARM_THUMB)
  add_compile_options(-mthumb)

  # GCC Manual: -mthumb ... If you want to force assembler files to be
  # interpreted as Thumb code, either add a `.thumb' directive to the source or
  # pass the -mthumb option directly to the assembler by prefixing it with -Wa.

  add_compile_options(-Wa,-mthumb)

  # Outputs an implicit IT block when there is a conditional instruction without
  # an enclosing IT block.

  add_compile_options(-Wa,-mimplicit-it=always)
endif()

# Debug link map

if(CONFIG_DEBUG_LINK_MAP)
  add_link_options(
    -Wl,--strict
    -Wl,--map
    -Wl,--xref
    -Wl,--symbols
    -Wl,--info=unused
    -Wl,--info=veneers
    -Wl,--info=summarysizes
    -Wl,--info=summarystack)
endif()

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(${CONFIG_DEBUG_SYMBOLS_LEVEL})

  add_link_options(-Wl,--debug)
endif()

add_compile_options(
  -Wno-attributes -Wno-unknown-pragmas
  $<$<COMPILE_LANGUAGE:C>:-Wstrict-prototypes>
  $<$<COMPILE_LANGUAGE:CXX>:-nostdinc++>)

# When all C++ code is built using GCC 7.1 or a higher version, we can safely
# disregard warnings of the type "parameter passing for X changed in GCC 7.1."
# Refer to :
# https://stackoverflow.com/questions/48149323/what-does-the-gcc-warning-project-parameter-passing-for-x-changed-in-gcc-7-1-m

add_compile_options(-Wno-psabi)

if(CONFIG_CXX_STANDARD)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=${CONFIG_CXX_STANDARD}>)
endif()

if(NOT CONFIG_CXX_EXCEPTION)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
                      $<$<COMPILE_LANGUAGE:CXX>:-fcheck-new>)
endif()

if(NOT CONFIG_CXX_RTTI)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>)
endif()

set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")

set(PREPROCESS ${CMAKE_C_COMPILER} ${CMAKE_C_FLAG_ARGS} -E -P -x c)

# ##############################################################################
# arch/arm/src/cmake/gcc.cmake
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

set(TOOLCHAIN_PREFIX arm-none-eabi)
set(CMAKE_LIBRARY_ARCHITECTURE ${TOOLCHAIN_PREFIX})
set(CMAKE_C_COMPILER_TARGET ${TOOLCHAIN_PREFIX})
set(CMAKE_CXX_COMPILER_TARGET ${TOOLCHAIN_PREFIX})

set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_PREPROCESSOR ${TOOLCHAIN_PREFIX}-gcc -E -P -x c)
set(CMAKE_STRIP ${TOOLCHAIN_PREFIX}-strip --strip-unneeded)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump)

if(NOT CONFIG_LTO_NONE AND CONFIG_ARM_TOOLCHAIN_GNU_EABI)
  set(CMAKE_LINKER ${TOOLCHAIN_PREFIX}-gcc)
  set(CMAKE_LD ${TOOLCHAIN_PREFIX}-gcc)
  set(CMAKE_AR ${TOOLCHAIN_PREFIX}-gcc-ar)
  set(CMAKE_NM ${TOOLCHAIN_PREFIX}-gcc-nm)
  set(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}-gcc-ranlib)
  add_compile_options(-fno-builtin)
else()
  set(CMAKE_LINKER ${TOOLCHAIN_PREFIX}-ld)
  set(CMAKE_LD ${TOOLCHAIN_PREFIX}-ld)
  set(CMAKE_AR ${TOOLCHAIN_PREFIX}-ar)
  set(CMAKE_NM ${TOOLCHAIN_PREFIX}-nm)
  set(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}-ranlib)
endif()

set(NO_LTO "-fno-lto")

# Workaround to skip -Warray-bounds check due to bug of GCC-12: Wrong warning
# array subscript [0] is outside array bounds:
# https://gcc.gnu.org/bugzilla/show_bug.cgi?id=105523

if(CONFIG_ARCH_TOOLCHAIN_GNU AND NOT CONFIG_ARCH_TOOLCHAIN_CLANG)
  execute_process(COMMAND ${CMAKE_C_COMPILER} --version
                  OUTPUT_VARIABLE GCC_VERSION_OUTPUT)
  string(REGEX MATCH "([0-9]+)\\.[0-9]+" GCC_VERSION_REGEX
               "${GCC_VERSION_OUTPUT}")
  set(GCCVER ${CMAKE_MATCH_1})

  if(GCCVER GREATER_EQUAL 12)
    add_link_options(-Wl,--print-memory-usage)
    add_compile_options(--param=min-pagesize=0)
    if(CONFIG_ARCH_RAMFUNCS OR NOT CONFIG_BOOT_RUNFROMFLASH)
      add_link_options(-Wl,--no-warn-rwx-segments)
    endif()
  endif()
endif()

# override the responsible file flag

if(CMAKE_GENERATOR MATCHES "Ninja")
  set(CMAKE_C_RESPONSE_FILE_FLAG "$DEFINES $INCLUDES $FLAGS @")
  set(CMAKE_CXX_RESPONSE_FILE_FLAG "$DEFINES $INCLUDES $FLAGS @")
  set(CMAKE_ASM_RESPONSE_FILE_FLAG "$DEFINES $INCLUDES $FLAGS @")
endif()

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
  add_compile_options(-fprofile-arcs -ftest-coverage -fno-inline)
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

if(CONFIG_PROFILE_ALL)
  add_compile_options(-pg)
endif()

if(CONFIG_UNWINDER_ARM)
  add_compile_options(-funwind-tables -fasynchronous-unwind-tables)
endif()

# Link Time Optimization

if(CONFIG_LTO_THIN)
  add_compile_options(-flto=thin)
elseif(CONFIG_LTO_FULL)
  add_compile_options(-flto)
  add_compile_options(-fuse-linker-plugin)
elseif(CONFIG_LTO_FAT)
  add_compile_options(-flto -ffat-lto-objects)
endif()

# The arm clang toolchain requires to pass the linker option will gcc tool chain
# can automatically perform lto at linking time if it found any object files are
# compiled with flto

if(NOT CONFIG_LTO_NONE)

  # For gcc, use the linker plugin to extract objects with GIMPLE info from the
  # lib archive

  add_compile_options(-fuse-linker-plugin)
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

# Optimization of unused sections

if(CONFIG_DEBUG_OPT_UNUSED_SECTIONS)
  add_link_options(-Wl,--gc-sections)
  add_compile_options(-ffunction-sections -fdata-sections)
endif()

# Debug --whole-archive

if(CONFIG_DEBUG_LINK_WHOLE_ARCHIVE)
  add_link_options(-Wl,--whole-archive)
endif()

# Debug link map

if(CONFIG_DEBUG_LINK_MAP)
  add_link_options(-Wl,--cref -Wl,-Map=nuttx.map)
endif()

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(${CONFIG_DEBUG_SYMBOLS_LEVEL})
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

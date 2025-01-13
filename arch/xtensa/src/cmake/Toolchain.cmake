# ##############################################################################
# arch/xtensa/src/cmake/Toolchain.cmake
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

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

if(CONFIG_XTENSA_TOOLCHAIN_XCC OR CONFIG_XTENSA_TOOLCHAIN_XCLANG)
  set(TOOLCHAIN_PREFIX xt)
endif()

if(CONFIG_XTENSA_TOOLCHAIN_ESP)
  set(TOOLCHAIN_PREFIX xtensa-${CONFIG_ARCH_CHIP}-elf-)
endif()

# Default toolchain
if(CONFIG_XTENSA_TOOLCHAIN_XCC)

  set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-xcc)
  set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
  set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-xc++)
  set(CMAKE_PREPROCESSOR ${TOOLCHAIN_PREFIX}-xcc -E -P -x c)

  add_compile_options(-Wno-atomic-alignment)

elseif(CONFIG_XTENSA_TOOLCHAIN_XCLANG)
  set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-clang)
  set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
  set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-clang++)
  set(CMAKE_PREPROCESSOR ${TOOLCHAIN_PREFIX}-clang -E -P -x c)
else()
  set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
  set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
  set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
  set(CMAKE_PREPROCESSOR ${TOOLCHAIN_PREFIX}-gcc -E -P -x c)
endif()

set(CMAKE_STRIP ${TOOLCHAIN_PREFIX}-strip --strip-unneeded)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump)
set(CMAKE_LD ${TOOLCHAIN_PREFIX}-ld)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}-ar)
set(CMAKE_NM ${TOOLCHAIN_PREFIX}-nm)
set(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}-ranlib)

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

add_compile_options(-mlongcalls)

add_compile_options(-mtext-section-literals)

if(CONFIG_MM_KASAN_INSTRUMENT_ALL)
  add_compile_options(-fsanitize=kernel-address)
endif()

if(CONFIG_MM_KASAN_DISABLE_READS_CHECK)
  add_compile_options(--param=asan-instrument-reads=0)
endif()

if(CONFIG_MM_KASAN_DISABLE_READS_CHECK)
  add_compile_options(--param=asan-instrument-writes=0)
endif()

if(CONFIG_MM_UBSAN_ALL)
  add_compile_options(${CONFIG_MM_UBSAN_OPTION})
endif()

if(CONFIG_MM_UBSAN_TRAP_ON_ERROR)
  add_compile_options(-fsanitize-undefined-trap-on-error)
endif()

if(CONFIG_DEBUG_CUSTOMOPT)
  add_compile_options(${CONFIG_DEBUG_OPTLEVEL})
elseif(CONFIG_DEBUG_FULLOPT)
  if(CONFIG_ARCH_TOOLCHAIN_CLANG)
    add_compile_options(-Oz)
  else()
    add_compile_options(-Os)
  endif()
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

if(${CONFIG_STACK_USAGE_WARNING})
  if(NOT ${CONFIG_STACK_USAGE_WARNING} STREQUAL 0)
    add_compile_options(-Wstack-usage=${CONFIG_STACK_USAGE_WARNING})
  endif()
endif()

if(CONFIG_ARCH_INSTRUMENT_ALL)
  add_compile_options(-finstrument-functions)
endif()

add_compile_options(
  -fno-common
  -Wall
  -Wshadow
  -Wundef
  -Wno-attributes
  -Wno-unknown-pragmas
  $<$<COMPILE_LANGUAGE:C>:-Wstrict-prototypes>
  $<$<COMPILE_LANGUAGE:CXX>:-nostdinc++>)

if(NOT ${CONFIG_ARCH_TOOLCHAIN_CLANG})
  add_compile_options(-Wno-psabi)
endif()

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

if(CONFIG_DEBUG_OPT_UNUSED_SECTIONS)
  add_link_options(-Wl,--gc-sections)
  add_compile_options(-ffunction-sections -fdata-sections)
endif()

# Debug --whole-archive
if(CONFIG_DEBUG_LINK_WHOLE_ARCHIVE)
  add_link_options(-Wl,--whole-archive)
endif()

add_link_options(-nostdlib)
add_link_options(-Wl,--entry=__start)

if(CONFIG_DEBUG_LINK_MAP)
  add_link_options(-Wl,--cref -Wl,-Map=nuttx.map)
endif()

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(${CONFIG_DEBUG_SYMBOLS_LEVEL})
endif()

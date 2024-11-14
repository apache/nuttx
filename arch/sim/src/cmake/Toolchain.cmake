# ##############################################################################
# arch/sim/src/cmake/Toolchain.cmake
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

if(APPLE)
  find_program(CMAKE_C_ELF_COMPILER x86_64-elf-gcc)
  find_program(CMAKE_CXX_ELF_COMPILER x86_64-elf-g++)
endif()

if(WIN32)
  return()
endif()

# NuttX is sometimes built as a native target. In that case, the __NuttX__ macro
# is predefined by the compiler. https://github.com/NuttX/buildroot
#
# In other cases, __NuttX__ is an ordinary user-definded macro. It's especially
# the case for NuttX sim, which is a target to run the entire NuttX as a program
# on the host OS, which can be Linux, macOS, Windows, etc.
# https://cwiki.apache.org/confluence/display/NUTTX/NuttX+Simulation In that
# case, the host OS compiler is used to build NuttX. Thus, eg. NuttX sim on
# macOS is built with __APPLE__. We #undef predefined macros for those possible
# host OSes here because the OS APIs this library should use are of NuttX, not
# the host OS.

set(SIM_NO_HOST_OPTIONS
    -U_AIX
    -U_WIN32
    -U__APPLE__
    -U__FreeBSD__
    -U__NetBSD__
    -U__linux__
    -U__sun__
    -U__unix__
    -U__ENVIRONMENT_MAC_OS_X_VERSION_MIN_REQUIRED__)

add_compile_options(${SIM_NO_HOST_OPTIONS})

set(NO_LTO "-fno-lto")

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(${CONFIG_DEBUG_SYMBOLS_LEVEL})
endif()

if(CONFIG_DEBUG_CUSTOMOPT)
  add_compile_options(${CONFIG_DEBUG_OPTLEVEL})
elseif(CONFIG_DEBUG_FULLOPT)
  add_compile_options(-O2)
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

if(CONFIG_STACK_USAGE_WARNING)
  add_compile_options(-Wstack-usage=${CONFIG_STACK_USAGE_WARNING})
endif()

if(CONFIG_COVERAGE_ALL)
  add_compile_options(-fprofile-generate -ftest-coverage)
endif()

if(CONFIG_PROFILE_ALL OR CONFIG_SIM_PROFILE)
  add_compile_options(-pg)
endif()

if(CONFIG_SIM_ASAN)
  add_compile_options(-fsanitize=address)
  add_link_options(-fsanitize=address)
  add_compile_options(-fsanitize-address-use-after-scope)
  add_compile_options(-fsanitize=pointer-compare)
  add_compile_options(-fsanitize=pointer-subtract)
  add_link_options(-fsanitize=address)
elseif(CONFIG_MM_KASAN_ALL)
  add_compile_options(-fsanitize=kernel-address)
endif()

if(CONFIG_MM_KASAN_GLOBAL)
  add_compile_options(--param=asan-globals=1)
endif()

if(CONFIG_SIM_UBSAN)
  add_link_options(-fsanitize=undefined)
  add_compile_options(-fsanitize=undefined)
  add_link_options(-fsanitize=undefined)
else()
  if(CONFIG_MM_UBSAN_ALL)
    add_compile_options(${CONFIG_MM_UBSAN_OPTION})
  endif()

  if(CONFIG_MM_UBSAN_TRAP_ON_ERROR)
    add_compile_options(-fsanitize-undefined-trap-on-error)
  endif()
endif()

if(CONFIG_DEBUG_OPT_UNUSED_SECTIONS)
  if(APPLE)
    add_link_options(-Wl,-dead_strip)
  else()
    add_link_options(-Wl,--gc-sections)
  endif()
  add_compile_options(-ffunction-sections -fdata-sections)
endif()

if(CONFIG_ARCH_INSTRUMENT_ALL)
  add_compile_options(-finstrument-functions)
endif()

if(NOT WIN32)
  # Add -fvisibility=hidden Because we don't want export nuttx's symbols to
  # shared libraries Add -fno-common because macOS "ld -r" doesn't seem to pick
  # objects for common symbols.
  add_compile_options(
    -fno-common
    -fvisibility=hidden
    -ffunction-sections
    -fdata-sections
    -Wall
    -Wshadow
    -Wundef
    -Wno-attributes
    -Wno-unknown-pragmas
    $<$<COMPILE_LANGUAGE:C>:-Wstrict-prototypes>)
else()
  add_compile_options(/std:c11 /experimental:c11atomics)
endif()

if(APPLE)
  add_compile_options(-Wno-deprecated-declarations)
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

if(NOT CONFIG_LIBCXXTOOLCHAIN)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-nostdinc++>)
endif()

if(CONFIG_SIM_M32)
  add_compile_options(-m32)
  add_link_options(-m32)
endif()

if(CONFIG_LIBCXX)
  if(APPLE)
    add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-DLIBCXX_BUILDING_LIBCXXABI>)
  endif()
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-D__GLIBCXX__>)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-D_LIBCPP_DISABLE_AVAILABILITY>)
endif()

if(APPLE)
  add_link_options(-Wl,-dead_strip)
else()
  add_link_options(-Wl,--gc-sections)
  add_link_options(-Wl,-Ttext-segment=0x40000000)
endif()

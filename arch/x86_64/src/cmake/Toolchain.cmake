# ##############################################################################
# arch/x86_64/src/cmake/Toolchain.cmake
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

set(ARCH_SUBDIR intel64)

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

if(CONFIG_DEBUG_CUSTOMOPT)
  add_compile_options(${CONFIG_DEBUG_OPTLEVEL})
elseif(CONFIG_DEBUG_FULLOPT)
  add_compile_options(-Os)
endif()

if(NOT CONFIG_DEBUG_NOOPT)
  add_compile_options(-fno-strict-aliasing)
endif()

# NOTE: don't set -fomit-frame-pointer - it breaks debugging with gdb. The
# addresses of local variables are shifted in gdb if this option is enabled

if(CONFIG_FRAME_POINTER)
  add_compile_options(-fno-omit-frame-pointer -fno-optimize-sibling-calls)
endif()

if(CONFIG_STACK_CANARIES)
  add_compile_options(-fstack-protector-all)
else()
  add_compile_options(-fno-stack-protector)
endif()

if(CONFIG_STACK_USAGE)
  add_compile_options(-fstack-usage)
endif()

if(${CONFIG_STACK_USAGE_WARNING})
  if(NOT ${CONFIG_STACK_USAGE_WARNING} STREQUAL 0)
    add_compile_options(-Wstack-usage=${CONFIG_STACK_USAGE_WARNING})
  endif()
endif()

if(CONFIG_COVERAGE_ALL)
  add_compile_options(-fprofile-generate -ftest-coverage)
endif()

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(${CONFIG_DEBUG_SYMBOLS_LEVEL})
endif()

if(CONFIG_HOST_LINUX)
  add_link_options(-Wl,-z,noexecstack)
endif()

# Architecture flags

add_link_options(-Wl,--entry=__pmode_entry)
add_link_options(-z max-page-size=0x1000)
add_link_options(-no-pie -nostdlib)
add_link_options(-Wl,--no-relax)
add_compile_options(-fno-pic -mcmodel=large)
add_compile_options(-mno-red-zone)

add_compile_options(
  -U_AIX
  -U_WIN32
  -U__APPLE__
  -U__FreeBSD__
  -U__NetBSD__
  -U__linux__
  -U__sun__
  -U__unix__
  -U__ENVIRONMENT_MAC_OS_X_VERSION_MIN_REQUIRED__)

if(CONFIG_DEBUG_LINK_MAP)
  add_link_options(-Wl,--cref -Wl,-Map=nuttx.map)
endif()

add_compile_options(
  -fno-common
  -Wall
  -Wshadow
  -Wundef
  -Wno-attributes
  -Wno-unknown-pragmas
  $<$<COMPILE_LANGUAGE:C>:-Wstrict-prototypes>)

if(CONFIG_CXX_STANDARD)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=${CONFIG_CXX_STANDARD}>)
endif()

if(CONFIG_LIBCXX)
  add_compile_options(-D_LIBCPP_DISABLE_AVAILABILITY)
endif()

if(NOT CONFIG_LIBCXXTOOLCHAIN)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-nostdinc++>)
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

if(CONFIG_DEBUG_LINK_WHOLE_ARCHIVE)
  add_link_options(-Wl,--whole-archive)
endif()

if(CONFIG_ARCH_INTEL64_HAVE_RDRAND)
  add_compile_options(-mrdrnd)
endif()

if(CONFIG_ARCH_X86_64_SSE3)
  add_compile_options(-msse3)
endif()

if(CONFIG_ARCH_X86_64_SSSE3)
  add_compile_options(-mssse3)
endif()

if(CONFIG_ARCH_X86_64_SSE41)
  add_compile_options(-msse4.1)
endif()

if(CONFIG_ARCH_X86_64_SSE42)
  add_compile_options(-msse4.2)
endif()

if(CONFIG_ARCH_X86_64_SSE4A)
  add_compile_options(-msse4a)
endif()

if(CONFIG_ARCH_X86_64_AVX)
  add_compile_options(-mavx)
endif()

if(CONFIG_ARCH_X86_64_AVX512)
  add_compile_options(-mavx512f)
endif()

if(CONFIG_ARCH_X86_64_AVX512PF)
  add_compile_options(-mavx512pf)
endif()

if(CONFIG_ARCH_X86_64_AVX512ER)
  add_compile_options(-mavx512er)
endif()

if(CONFIG_ARCH_X86_64_AVX512CD)
  add_compile_options(-mavx512cd)
endif()

if(CONFIG_ARCH_X86_64_AVX512VL)
  add_compile_options(-mavx512vl)
endif()

if(CONFIG_ARCH_X86_64_AVX512BW)
  add_compile_options(-mavx512bw)
endif()

if(CONFIG_ARCH_X86_64_AVX512DQ)
  add_compile_options(-mavx512dq)
endif()

if(CONFIG_ARCH_X86_64_AVX512IFMA)
  add_compile_options(-mavx512ifma)
endif()

if(CONFIG_ARCH_X86_64_AVX512VBMI)
  add_compile_options(-mavx512vbmi)
endif()

if(CONFIG_ARCH_TOOLCHAIN_GNU AND NOT CONFIG_ARCH_TOOLCHAIN_CLANG)
  if(NOT GCCVER)
    execute_process(COMMAND ${CMAKE_C_COMPILER} --version
                    OUTPUT_VARIABLE GCC_VERSION_OUTPUT)
    string(REGEX MATCH "[0-9]+\\.[0-9]+" GCC_VERSION_REGEX
                 "${GCC_VERSION_OUTPUT}")
    set(GCCVER ${CMAKE_MATCH_1})
  endif()
endif()

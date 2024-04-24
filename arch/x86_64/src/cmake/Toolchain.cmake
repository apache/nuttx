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

set(CMAKE_C_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_ASM_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")

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

if(CONFIG_ARCH_COVERAGE)
  add_compile_options(-fprofile-generate -ftest-coverage)
endif()

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(-g)
endif()

# Architecture flags

add_link_options(-Wl,--entry=__pmode_entry)
add_link_options(-z max-page-size=0x1000)
add_link_options(-no-pie -nostdlib)
add_link_options(-Wl,--no-relax)
add_compile_options(-fPIC)
add_compile_options(-mno-red-zone)

if(CONFIG_DEBUG_LINK_MAP)
  add_link_options(-Wl,--cref -Wl,-Map=nuttx.map)
endif()

set(ARCHCFLAGS
    "-Wstrict-prototypes -fno-common -Wall -Wshadow -Wundef -Wno-attributes -Wno-unknown-pragmas"
)
set(ARCHCXXFLAGS
    "-fno-common -Wall -Wshadow -Wundef -Wno-attributes -Wno-unknown-pragmas")

if(NOT CONFIG_LIBCXXTOOLCHAIN)
  set(ARCHCXXFLAGS "${ARCHCXXFLAGS} -nostdinc++")
endif()

if(NOT CONFIG_CXX_EXCEPTION)
  string(APPEND ARCHCXXFLAGS " -fno-exceptions -fcheck-new")
endif()

if(NOT CONFIG_CXX_RTTI)
  string(APPEND ARCHCXXFLAGS " -fno-rtti")
endif()

if(NOT "${CMAKE_C_FLAGS}" STREQUAL "")
  string(REGEX MATCH "${ARCHCFLAGS}" EXISTS_FLAGS "${CMAKE_C_FLAGS}")
endif()

if(NOT EXISTS_FLAGS)
  set(CMAKE_ASM_FLAGS
      "${CMAKE_ASM_FLAGS} ${ARCHCFLAGS}"
      CACHE STRING "" FORCE)
  set(CMAKE_C_FLAGS
      "${CMAKE_C_FLAGS} ${ARCHCFLAGS}"
      CACHE STRING "" FORCE)
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} ${ARCHCXXFLAGS}"
      CACHE STRING "" FORCE)
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

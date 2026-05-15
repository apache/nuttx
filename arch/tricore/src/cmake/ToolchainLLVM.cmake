# ##############################################################################
# arch/tricore/src/cmake/ToolchainLLVM.cmake
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

# CMake's compiler check links a test executable with linker flags it selects
# for the host's default ld.  ld.lld rejects the GNU-ld-only
# --no-warn-rwx-segments option, so skip the link step entirely.

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if(CONFIG_ARCH_CHIP_FAMILY_TC3X)
  set(ARCH_SUBDIR tc3x)
elseif(CONFIG_ARCH_CHIP_FAMILY_TC4X)
  set(ARCH_SUBDIR tc4x)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/chip.cmake)

set(CMAKE_ASM_COMPILER clang)
set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)
set(CMAKE_STRIP llvm-strip --strip-unneeded)
set(CMAKE_OBJCOPY llvm-objcopy)
set(CMAKE_OBJDUMP llvm-objdump)

set(CMAKE_LINKER clang)
set(CMAKE_LD clang)
set(CMAKE_AR llvm-ar)
set(CMAKE_NM llvm-nm)
set(CMAKE_RANLIB llvm-ranlib)

set(CMAKE_C_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_ASM_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")

add_compile_options(-fno-common)
add_compile_options(-Wall -Wshadow -Wundef)
add_compile_options(-Wno-unused-command-line-argument)
add_compile_options(-nostdlib)

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

if(CONFIG_DEBUG_OPT_UNUSED_SECTIONS)
  add_link_options(-Wl,--gc-sections)
  add_compile_options(-ffunction-sections -fdata-sections)
endif()

if(CONFIG_DEBUG_LINK_WHOLE_ARCHIVE)
  add_link_options(-Wl,--whole-archive)
endif()

if(CONFIG_LTO_THIN)
  add_compile_options(-flto=thin)
elseif(CONFIG_LTO_FULL)
  add_compile_options(-flto)
endif()

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(-g)
endif()

add_compile_options(-Wno-attributes -Wno-unknown-pragmas
                    $<$<COMPILE_LANGUAGE:C>:-Wstrict-prototypes>)

if(CONFIG_CXX_STANDARD)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=${CONFIG_CXX_STANDARD}>)
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

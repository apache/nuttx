# ##############################################################################
# arch/arm64/src/cmake/Toolchain.cmake
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

# Default toolchain
set(TOOLCHAIN_PREFIX aarch64-none-elf)

set(CMAKE_LIBRARY_ARCHITECTURE ${TOOLCHAIN_PREFIX})
set(CMAKE_C_COMPILER_TARGET ${TOOLCHAIN_PREFIX})
set(CMAKE_CXX_COMPILER_TARGET ${TOOLCHAIN_PREFIX})
set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_STRIP ${TOOLCHAIN_PREFIX}-strip --strl dunneeded)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump)
set(CMAKE_LINKER ${TOOLCHAIN_PREFIX}-ld)
set(CMAKE_LD ${TOOLCHAIN_PREFIX}-ld)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}-ar)
set(CMAKE_NM ${TOOLCHAIN_PREFIX}-nm)
set(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}-gcc-ranlib)
if(CONFIG_LTO_FULL)
  add_compile_options(-flto)
  if(CONFIG_ARM64_TOOLCHAIN_GNU_EABI)
    set(CMAKE_LD ${TOOLCHAIN_PREFIX}-gcc)
    set(CMAKE_AR ${TOOLCHAIN_PREFIX}-gcc-ar)
    set(CMAKE_NM ${TOOLCHAIN_PREFIX}-gcc-nm)
    add_compile_options(-fuse-linker-plugin)
    add_compile_options(-fno-builtin)
  endif()
endif()

add_link_options(--entry=__start)
# override the ARCHIVE command
set(CMAKE_C_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_ASM_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")

if(CONFIG_ARCH_ARMV8A)
  add_compile_options(-march=armv8-a)
endif()

if(CONFIG_ARCH_ARMV8R)
  add_compile_options(-march=armv8-r)
endif()

if(CONFIG_ARCH_CORTEX_A53)
  add_compile_options(-mcpu=cortex-a53)
elseif(CONFIG_ARCH_CORTEX_A57)
  add_compile_options(-mcpu=cortex-a57)
elseif(CONFIG_ARCH_CORTEX_A72)
  add_compile_options(-mcpu=cortex-a72)
elseif(CONFIG_ARCH_CORTEX_R82)
  add_compile_options(-mcpu=cortex-r82)
endif()

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

if(CONFIG_ARCH_COVERAGE_ALL)
  add_compile_options(-fprofile-generate -ftest-coverage)
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

if(CONFIG_ARCH_INSTRUMENT_ALL)
  add_compile_options(-finstrument-functions)
endif()

if(CONFIG_ARCH_FPU)
  add_compile_options(-D_LDBL_EQ_DBL)
endif()

set(ARCHCFLAGS
    "-Wstrict-prototypes -fno-common -Wall -Wshadow -Werror -Wundef -Wno-attributes -Wno-unknown-pragmas"
)
set(ARCHCXXFLAGS
    "-fno-common -Wall -Wshadow -Wundef -Wno-attributes -Wno-unknown-pragmas")

if(NOT CONFIG_LIBCXXTOOLCHAIN)
  set(ARCHCXXFLAGS "${ARCHCXXFLAGS} -nostdinc++")
endif()

if(NOT ${CONFIG_ARCH_TOOLCHAIN_CLANG})
  string(APPEND ARCHCFLAGS " -Wno-psabi")
  string(APPEND ARCHCXXFLAGS " -Wno-psabi")
endif()

if(${CONFIG_CXX_STANDARD})
  string(APPEND ARCHCXXFLAGS " -std=${CONFIG_CXX_STANDARD}")
endif()

if(NOT ${CONFIG_CXX_EXCEPTION})
  string(APPEND ARCHCXXFLAGS " -fno-exceptions -fcheck-new")
endif()

if(NOT ${CONFIG_CXX_RTTI})
  string(APPEND ARCHCXXFLAGS " -fno-rtti")
endif()

add_link_options(-nostdlib)

if(CONFIG_DEBUG_OPT_UNUSED_SECTIONS)
  add_link_options(-Wl,--gc-sections)
  add_compile_options(-ffunction-sections -fdata-sections)
endif()

if(CONFIG_DEBUG_LINK_MAP)
  add_link_options(-Wl,-cref,-Map=nuttx.map)
endif()

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(-g)
endif()

if(CONFIG_ARCH_TOOLCHAIN_GNU)
  if(NOT GCCVER)

    execute_process(COMMAND ${CMAKE_C_COMPILER} --version
                    OUTPUT_VARIABLE GCC_VERSION_INFO)

    string(REGEX MATCH "[0-9]+\\.[0-9]+" GCC_VERSION ${GCC_VERSION_INFO})
    string(REGEX REPLACE "\\..*" "" GCCVER ${GCC_VERSION})

  endif()

  if(GCCVER EQUAL 12)
    add_link_options(-Wl,--no-warn-rwx-segments)
  endif()
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

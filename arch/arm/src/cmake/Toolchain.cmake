# ##############################################################################
# arch/arm/src/cmake/Toolchain.cmake
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

set(ARCH_SUBDIR)

if(CONFIG_ARCH_ARMV7A) # ARMv7-A
  set(ARCH_SUBDIR armv7-a)
elseif(CONFIG_ARCH_ARMV7R) # ARMv7-R
  set(ARCH_SUBDIR armv7-r)
elseif(CONFIG_ARCH_ARMV8R) # ARMv8-R
  set(ARCH_SUBDIR armv8-r)
elseif(CONFIG_ARCH_ARMV7M) # ARMv7-M
  set(ARCH_SUBDIR armv7-m)
elseif(CONFIG_ARCH_ARMV8M) # ARMv8-M
  set(ARCH_SUBDIR armv8-m)
elseif(CONFIG_ARCH_ARMV6M) # ARMv6-M
  set(ARCH_SUBDIR armv6-m)
else() # ARM9, ARM7TDMI, etc.
  set(ARCH_SUBDIR arm)
endif()

include(${ARCH_SUBDIR})

if(CONFIG_ARCH_TOOLCHAIN_CLANG)
  set(CMAKE_ASM_COMPILER clang)
  set(CMAKE_C_COMPILER clang)
  set(CMAKE_CXX_COMPILER clang++)
  set(CMAKE_STRIP llvm-strip --strip-unneeded)
  set(CMAKE_OBJCOPY llvm-objcopy)
  set(CMAKE_OBJDUMP llvm-objdump)
  set(CMAKE_LINKER ld.lld)
  set(CMAKE_LD ld.lld)
  set(CMAKE_AR llvm-ar)
  set(CMAKE_NM llvm-nm)
  set(CMAKE_RANLIB llvm-ranlib)

  # Since the no_builtin attribute is not fully supported on Clang disable the
  # built-in functions, refer:
  # https://github.com/apache/incubator-nuttx/pull/5971

  add_compile_options(-fno-builtin)
else()
  set(TOOLCHAIN_PREFIX arm-none-eabi)
  set(CMAKE_LIBRARY_ARCHITECTURE ${TOOLCHAIN_PREFIX})
  set(CMAKE_C_COMPILER_TARGET ${TOOLCHAIN_PREFIX})
  set(CMAKE_CXX_COMPILER_TARGET ${TOOLCHAIN_PREFIX})

  set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
  set(CMAKE_C_COMPILER ${CMAKE_ASM_COMPILER})
  set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
  set(CMAKE_STRIP ${TOOLCHAIN_PREFIX}-strip --strip-unneeded)
  set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
  set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump)

  if(CONFIG_LTO_FULL AND CONFIG_ARCH_TOOLCHAIN_GNU)
    set(CMAKE_LINKER ${TOOLCHAIN_PREFIX}-gcc)
    set(CMAKE_LD ${TOOLCHAIN_PREFIX}-gcc)
    set(CMAKE_AR ${TOOLCHAIN_PREFIX}-gcc-ar)
    set(CMAKE_NM ${TOOLCHAIN_PREFIX}-gcc-nm)
    set(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}-gcc-ranlib)
  else()
    set(CMAKE_LINKER ${TOOLCHAIN_PREFIX}-ld)
    set(CMAKE_LD ${TOOLCHAIN_PREFIX}-ld)
    set(CMAKE_AR ${TOOLCHAIN_PREFIX}-ar)
    set(CMAKE_NM ${TOOLCHAIN_PREFIX}-nm)
    set(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}-ranlib)
  endif()
endif()

# override the ARCHIVE command

set(CMAKE_C_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_ASM_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")

# Architecture flags

add_link_options(-Wl,--entry=__start)
add_link_options(-nostdlib)
add_compile_options(-fno-common)
add_compile_options(-Wall -Wshadow -Wundef)
add_compile_options(-nostdlib)

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

if(CONFIG_UNWINDER_ARM)
  add_compile_options(-funwind-tables -fasynchronous-unwind-tables)
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

if(CONFIG_ARCH_COVERAGE)
  add_compile_options(-fprofile-generate -ftest-coverage)
endif()

# Optimization of unused sections

if(CONFIG_DEBUG_OPT_UNUSED_SECTIONS)
  add_link_options(-Wl,--gc-sections)
  add_compile_options(-ffunction-sections -fdata-sections)
endif()

if(CONFIG_ENDIAN_BIG)
  add_compile_options(-mbig-endian)
endif()

# Link Time Optimization

if(CONFIG_LTO_THIN)
  add_compile_options(-flto=thin)
elseif(CONFIG_LTO_FULL)
  add_compile_options(-flto)
  if(CONFIG_ARCH_TOOLCHAIN_GNU)
    add_compile_options(-fno-builtin)
    add_compile_options(-fuse-linker-plugin)
  endif()
endif()

# Debug link map

if(CONFIG_DEBUG_LINK_MAP)
  add_link_options(-Wl,--cref -Wl,-Map=nuttx.map)
endif()

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(-g)
endif()

set(ARCHCFLAGS "-Wstrict-prototypes")

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

if(CONFIG_ARCH_TOOLCHAIN_CLANG)
  set(CMAKE_EXE_LINKER_FLAGS_INIT "-c")
else()
  set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")
endif()

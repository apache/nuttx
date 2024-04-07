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

  if(TOOLCHAIN_CLANG_CONFIG)
    execute_process(COMMAND clang --version
                    OUTPUT_VARIABLE clang_full_version_string)

    string(REGEX REPLACE ".*clang version ([0-9]+\\.[0-9]+).*" "\\1" CLANGVER
                         ${clang_full_version_string})

    if(CLANGVER STREQUAL "14.0")
      set(TOOLCHAIN_CLANG_CONFIG ${TOOLCHAIN_CLANG_CONFIG}_nosys)
    endif()
    add_compile_options(${TOOLCHAIN_CLANG_CONFIG}.cfg)

  endif()

elseif(CONFIG_ARM_TOOLCHAIN_ARMCLANG)
  set(CMAKE_ASM_COMPILER armclang)
  set(CMAKE_C_COMPILER armclang)
  set(CMAKE_CXX_COMPILER armclang)
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
  # Input sections are specified even though there will be no such sections
  # found in the libraries linked. Warning: L6314W: No section matches pattern
  # *(xxx).

  add_link_options(-Wl,--diag_suppress=6314)

  # Allow Empty Execution region declared on scatter Warning: L6312W: Empty
  # Execution region description for region xxx

  add_link_options(-Wl,--diag_suppress=6312)

  # Match pattern for an unused section that is being removed. Warning: L6329W:
  # Pattern xxx only matches removed unused sections.

  add_link_options(-Wl,--diag_suppress=6329)

else()
  set(TOOLCHAIN_PREFIX arm-none-eabi)
  set(CMAKE_LIBRARY_ARCHITECTURE ${TOOLCHAIN_PREFIX})
  set(CMAKE_C_COMPILER_TARGET ${TOOLCHAIN_PREFIX})
  set(CMAKE_CXX_COMPILER_TARGET ${TOOLCHAIN_PREFIX})

  set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})
  set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
  set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
  set(CMAKE_STRIP ${TOOLCHAIN_PREFIX}-strip --strip-unneeded)
  set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
  set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump)

  if(CONFIG_LTO_FULL)
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

  # Workaround to skip -Warray-bounds check due to bug of GCC-12: Wrong warning
  # array subscript [0] is outside array bounds:
  # https://gcc.gnu.org/bugzilla/show_bug.cgi?id=105523

  execute_process(COMMAND ${CMAKE_C_COMPILER} --version
                  OUTPUT_VARIABLE GCC_VERSION_OUTPUT)
  string(REGEX MATCH "\\+\\+.* ([0-9]+)\\.[0-9]+" GCC_VERSION_REGEX
               "${GCC_VERSION_OUTPUT}")
  set(GCCVER ${CMAKE_MATCH_1})

  if(GCCVER EQUAL 12)
    add_compile_options(--param=min-pagesize=0)
    if(CONFIG_ARCH_RAMFUNCS)
      add_link_options(-Wl,--no-warn-rwx-segments)
    endif()
  endif()
endif()

# override the ARCHIVE command

set(CMAKE_C_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_ASM_ARCHIVE_CREATE "<CMAKE_AR> rcs <TARGET> <LINK_FLAGS> <OBJECTS>")

if(CONFIG_ENDIAN_BIG)
  add_compile_options(-mbig-endian)
endif()

if(CONFIG_ARCH_TOOLCHAIN_CLANG)
  add_compile_options(-fshort-enums)
endif()

# Architecture flags

add_link_options(-Wl,--entry=__start)
add_link_options(-nostdlib)
add_compile_options(-fno-common -Wall -Wshadow -Wundef -nostdlib)

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
if(CONFIG_STACK_USAGE_WARNING AND NOT "${CONFIG_STACK_USAGE_WARNING}" STREQUAL
                                  "0")
  add_compile_options(-Wstack-usage=${CONFIG_STACK_USAGE_WARNING})
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

if(CONFIG_MM_KASAN_DISABLE_READS_CHECK)
  add_compile_options(--param asan-instrument-reads=0)
endif()

if(CONFIG_MM_KASAN_DISABLE_WRITES_CHECK)
  add_compile_options(--param asan-instrument-writes=0)
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
  if(CONFIG_ARM_TOOLCHAIN_ARMCLANG)
    add_link_options(-Wl,--lto)
  endif()
elseif(CONFIG_LTO_FULL)
  add_compile_options(-flto)
  if(CONFIG_ARM_TOOLCHAIN_GNU_EABI)
    add_compile_options(-fuse-linker-plugin)
  endif()
  if(CONFIG_ARM_TOOLCHAIN_ARMCLANG)
    add_link_options(-Wl,--lto)
  endif()
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

if(NOT CONFIG_ARM_TOOLCHAIN_ARMCLANG)

  # Optimization of unused sections

  if(CONFIG_DEBUG_OPT_UNUSED_SECTIONS)
    add_link_options(-Wl,--gc-sections)
    add_compile_options(-ffunction-sections -fdata-sections)
  endif()
endif()

# Debug link map

if(CONFIG_DEBUG_LINK_MAP)
  if(NOT CONFIG_ARM_TOOLCHAIN_ARMCLANG)
    add_link_options(-Wl,--cref -Wl,-Map=nuttx.map)
  else()
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
endif()

if(CONFIG_DEBUG_SYMBOLS)
  add_compile_options(-g)
  if(CONFIG_ARM_TOOLCHAIN_ARMCLANG)
    add_link_options(-Wl,--debug)
  endif()
endif()

set(ARCHCFLAGS "-Wstrict-prototypes -Wno-attributes -Wno-unknown-pragmas")
set(ARCHCXXFLAGS "-Wno-attributes -Wno-unknown-pragmas")

# When all C++ code is built using GCC 7.1 or a higher version, we can safely
# disregard warnings of the type "parameter passing for X changed in GCC 7.1."
# Refer to :
# https://stackoverflow.com/questions/48149323/what-does-the-gcc-warning-project-parameter-passing-for-x-changed-in-gcc-7-1-m

if(NOT CONFIG_ARCH_TOOLCHAIN_CLANG)
  string(APPEND ARCHCFLAGS " -Wno-psabi")
  string(APPEND ARCHCXXFLAGS " -Wno-psabi")
endif()

if(CONFIG_CXX_STANDARD)
  string(APPEND ARCHCXXFLAGS " -std=${CONFIG_CXX_STANDARD}")
endif()

if(NOT CONFIG_LIBCXXTOOLCHAIN)
  set(ARCHCXXFLAGS "${ARCHCXXFLAGS} -nostdinc++")
else()
  set(ARCHCXXFLAGS "${ARCHCXXFLAGS} -D_STDLIB_H_")
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

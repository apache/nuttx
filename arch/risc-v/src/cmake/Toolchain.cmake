# ##############################################################################
# arch/risc-v/src/cmake/Toolchain.cmake
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

if(CONFIG_RISCV_TOOLCHAIN_GNU_RV32
   OR CONFIG_RISCV_TOOLCHAIN_GNU_RV64
   OR CONFIG_RISCV_TOOLCHAIN_GNU_RV64ILP32
   OR CONFIG_RISCV_TOOLCHAIN_CLANG)
  if(NOT CONFIG_RISCV_TOOLCHAIN)
    set(CONFIG_RISCV_TOOLCHAIN GNU_RVG)
  endif()
endif()

# Default toolchain

if(CONFIG_ARCH_TOOLCHAIN_CLANG)
  set(TOOLCHAIN_PREFIX riscv64-unknown-elf)
else()
  find_program(RV_COMPILER riscv-none-elf-gcc)
  if(RV_COMPILER)
    set(TOOLCHAIN_PREFIX riscv-none-elf)
  else()
    if(CONFIG_RISCV_TOOLCHAIN_GNU_RV32)
      set(TOOLCHAIN_PREFIX riscv32-unknown-elf)
    else()
      set(TOOLCHAIN_PREFIX riscv64-unknown-elf)
    endif()
  endif()
endif()

set(CMAKE_LIBRARY_ARCHITECTURE ${TOOLCHAIN_PREFIX})
set(CMAKE_C_COMPILER_TARGET ${TOOLCHAIN_PREFIX})
set(CMAKE_CXX_COMPILER_TARGET ${TOOLCHAIN_PREFIX})

if(CONFIG_ARCH_TOOLCHAIN_CLANG)
  set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-clang)
  set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-clang)
  set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-clang++)
  set(CMAKE_STRIP ${TOOLCHAIN_PREFIX}-llvm-strip --strip-unneeded)
  set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-llvm-objcopy)
  set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-llvm-objdump)
  set(CMAKE_LINKER ${TOOLCHAIN_PREFIX}-ld)
  set(CMAKE_LD ${TOOLCHAIN_PREFIX}-ld)
  set(CMAKE_AR ${TOOLCHAIN_PREFIX}-llvm-ar)
  set(CMAKE_NM ${TOOLCHAIN_PREFIX}-llvm-nm)
  set(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}-llvm-ranlib)

  # Since the no_builtin attribute is not fully supported on Clang disable the
  # built-in functions, refer:
  # https://github.com/apache/incubator-nuttx/pull/5971

  add_compile_options(-fno-builtin)
else()
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

# Link Time Optimization

if(CONFIG_LTO_THIN)
  add_compile_options(-flto=thin)
elseif(CONFIG_LTO_FULL)
  add_compile_options(-flto)
  if(CONFIG_ARCH_TOOLCHAIN_GNU)
    add_compile_options(-fno-builtin)
    add_compile_options(-fuse-linker-plugin)
    add_link_options(-wl,--print-memory-usage)
  endif()
endif()

set(NO_LTO "-fno-lto")

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

if(CONFIG_SCHED_GCOV_ALL)
  add_compile_options(-fprofile-generate -ftest-coverage)
endif()

add_compile_options(
  -fno-common
  -Wall
  -Wshadow
  -Wundef
  -Wno-attributes
  -Wno-unknown-pragmas
  $<$<COMPILE_LANGUAGE:C>:-Wstrict-prototypes>)

if(NOT CONFIG_LIBCXXTOOLCHAIN)
  add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-nostdinc++>)
endif()

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

if(CONFIG_ARCH_RV32)
  add_link_options(-Wl,-melf32lriscv)
elseif(CONFIG_ARCH_RV64)
  add_compile_options(-mcmodel=medany)
  if(CONFIG_ARCH_RV64ILP32)
    add_link_options(-Wl,-melf32lriscv)
  else()
    add_link_options(-Wl,-melf64lriscv)
  endif()
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

# Generic GNU RVG toolchain
if(CONFIG_RISCV_TOOLCHAIN STREQUAL GNU_RVG)

  set(ARCHCPUEXTFLAGS i)

  if(CONFIG_ARCH_RV_ISA_M)
    set(ARCHCPUEXTFLAGS ${ARCHCPUEXTFLAGS}m)
  endif()

  if(CONFIG_ARCH_RV_ISA_A)
    set(ARCHCPUEXTFLAGS ${ARCHCPUEXTFLAGS}a)
  endif()

  if(CONFIG_ARCH_FPU)
    set(ARCHCPUEXTFLAGS ${ARCHCPUEXTFLAGS}f)
  endif()

  if(CONFIG_ARCH_DPFPU)
    set(ARCHCPUEXTFLAGS ${ARCHCPUEXTFLAGS}d)
  endif()

  if(CONFIG_ARCH_QPFPU)
    set(ARCHCPUEXTFLAGS ${ARCHCPUEXTFLAGS}q)
  endif()

  if(CONFIG_ARCH_RV_ISA_C)
    set(ARCHCPUEXTFLAGS ${ARCHCPUEXTFLAGS}c)
  endif()

  if(CONFIG_ARCH_RV_ISA_V)
    set(ARCHCPUEXTFLAGS ${ARCHCPUEXTFLAGS}v)
  endif()

  if(NOT GCCVER)
    execute_process(COMMAND ${CMAKE_CXX_COMPILER} --version
                    OUTPUT_VARIABLE GCC_VERSION_OUTPUT)
    string(REGEX MATCH "([0-9]+)\\.[0-9]+" GCC_VERSION_REGEX
                 "${GCC_VERSION_OUTPUT}")
    set(GCCVER ${CMAKE_MATCH_1})

    if(GCCVER GREATER_EQUAL 12)
      if(CONFIG_ARCH_RAMFUNCS OR NOT CONFIG_BOOT_RUNFROMFLASH)
        add_link_options(-Wl,--no-warn-rwx-segments)
      endif()
    endif()
  endif()

  if(CONFIG_ARCH_RV_ISA_ZICSR_ZIFENCEI)
    if(GCCVER GREATER_EQUAL 12 OR CONFIG_ARCH_TOOLCHAIN_CLANG)
      set(ARCHCPUEXTFLAGS ${ARCHCPUEXTFLAGS}_zicsr_zifencei)
    endif()
  endif()

  if(CONFIG_ARCH_RV_EXPERIMENTAL_EXTENSIONS)
    set(ARCHCPUEXTFLAGS
        ${ARCHCPUEXTFLAGS}_${CONFIG_ARCH_RV_EXPERIMENTAL_EXTENSIONS})
    add_compile_options(-menable-experimental-extensions)
  endif()

  if(CONFIG_ARCH_RV_ISA_VENDOR_EXTENSIONS)
    set(ARCHCPUEXTFLAGS
        ${ARCHCPUEXTFLAGS}_${CONFIG_ARCH_RV_ISA_VENDOR_EXTENSIONS})
  endif()

  # Detect abi type

  if(CONFIG_ARCH_RV32)
    set(ARCHTYPE "rv32")
    set(ARCHABITYPE "ilp32")
    set(LLVM_ARCHTYPE "riscv32")
  elseif(CONFIG_ARCH_RV64)
    set(ARCHTYPE "rv64")
    if(CONFIG_ARCH_RV64ILP32)
      set(ARCHABITYPE "ilp32")
    else()
      set(ARCHABITYPE "lp64")
    endif()
    set(LLVM_ARCHTYPE "riscv64")
  endif()

  # Construct arch flags

  set(ARCHCPUFLAGS -march=${ARCHTYPE}${ARCHCPUEXTFLAGS})

  # Construct arch abi flags

  if(CONFIG_ARCH_DPFPU)
    list(APPEND ARCHCPUFLAGS -mabi=${ARCHABITYPE}d)
    set(LLVM_ABITYPE ${ARCHABITYPE}d)
  elseif(CONFIG_ARCH_FPU)
    list(APPEND ARCHCPUFLAGS -mabi=${ARCHABITYPE}f)
    set(LLVM_ABITYPE ${ARCHABITYPE}f)
  else()
    list(APPEND ARCHCPUFLAGS -mabi=${ARCHABITYPE})
    set(LLVM_ABITYPE ${ARCHABITYPE})
  endif()

  # RISCV has a modular instruction set. It's hard to define cpu-model to
  # support all toolchain. For Zig, cpu model is this formal:
  # generic_rv[32|64][i][m][a][f][d][c] For Rust, cpu model is this formal:
  # riscv[32|64][i][m][a][f][d][c] So, it's better to map the NuttX config to
  # LLVM builtin cpu model, these models supported by all LLVM based toolchain.
  # Refer to :
  # https://github.com/llvm/llvm-project/blob/release/15.x/llvm/lib/Target/RISCV/RISCV.td
  # These models can't cover all implementation of RISCV, but it's enough for
  # most cases.

  set(LLVM_CPUFLAGS)

  if(CONFIG_ARCH_RV32)
    if(${ARCHCPUEXTFLAGS} STREQUAL imc)
      list(APPEND LLVM_CPUFLAGS -mcpu=sifive-e20)
    elseif(${ARCHCPUEXTFLAGS} STREQUAL imac)
      list(APPEND LLVM_CPUFLAGS -mcpu=sifive-e31)
    elseif(${ARCHCPUEXTFLAGS} STREQUAL imafc)
      list(APPEND LLVM_CPUFLAGS -mcpu=sifive-e76)
    endif()
  else()
    if(${ARCHCPUEXTFLAGS} STREQUAL imac)
      list(APPEND LLVM_CPUFLAGS -mcpu=sifive-s51)
    elseif(${ARCHCPUEXTFLAGS} STREQUAL imafdc)
      list(APPEND LLVM_CPUFLAGS -mcpu=sifive-u54)
    endif()
  endif()

  list(APPEND PLATFORM_FLAGS ${ARCHCPUFLAGS})

  add_compile_options(${PLATFORM_FLAGS})

endif()

if(CONFIG_MM_KASAN_ALL)
  add_compile_options(-fsanitize=kernel-address)
endif()

if(CONFIG_MM_KASAN_DISABLE_READS_CHECK)
  add_compile_options(--param=asan-instrument-reads=0)
endif()

if(CONFIG_MM_KASAN_DISABLE_WRITES_CHECK)
  add_compile_options(--param=asan-instrument-writes=0)
endif()

if(CONFIG_MM_UBSAN_ALL)
  add_compile_options(${CONFIG_MM_UBSAN_OPTION})
endif()

if(CONFIG_MM_UBSAN_TRAP_ON_ERROR)
  add_compile_options(-fsanitize-undefined-trap-on-error)
endif()

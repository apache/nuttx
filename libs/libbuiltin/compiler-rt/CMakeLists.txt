# ##############################################################################
# libs/libbuiltin/compiler-rt/CMakeLists.txt
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

if(CONFIG_LIB_COMPILER_RT)

  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/compiler-rt)

    set(COMPILER_RT_VERSION ${CONFIG_LIB_COMPILER_RT_VERSION})

    FetchContent_Declare(
      compiler-rt
      DOWNLOAD_NAME "compiler-rt-${COMPILER_RT_VERSION}.src.tar.xz"
      DOWNLOAD_DIR ${CMAKE_CURRENT_LIST_DIR}
      URL "https://github.com/llvm/llvm-project/releases/download/llvmorg-${COMPILER_RT_VERSION}/compiler-rt-${COMPILER_RT_VERSION}.src.tar.xz"
          SOURCE_DIR
          ${CMAKE_CURRENT_LIST_DIR}/compiler-rt
          BINARY_DIR
          ${CMAKE_BINARY_DIR}/libs/libbuiltin/compiler-rt
          CONFIGURE_COMMAND
          ""
          BUILD_COMMAND
          ""
          INSTALL_COMMAND
          ""
          TEST_COMMAND
          ""
      DOWNLOAD_NO_PROGRESS true
      TIMEOUT 100)

    FetchContent_GetProperties(compiler-rt)

    if(NOT compiler-rt_POPULATED)
      FetchContent_Populate(compiler-rt)
    endif()

  endif()

endif()

if(CONFIG_ARCH_ARM)
  set(ARCH arm)
elseif(CONFIG_ARCH_RISCV)
  set(ARCH riscv)
elseif(CONFIG_ARCH_X86_64)
  set(ARCH x86_64)
elseif(CONFIG_ARCH_ARM64)
  set(ARCH aarch64)
endif()

list(APPEND INCDIR ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/include)

if(CONFIG_BUILTIN_COMPILER_RT)

  nuttx_add_system_library(rt.builtins)

  target_include_directories(
    rt.builtins PRIVATE ${INCDIR}
                        ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/builtins)

  target_compile_options(rt.builtins PRIVATE -Wno-undef -Wno-macro-redefined)

  set(SRCSTMP)
  set(RT_BUILTINS_SRCS)
  set(BUILTINS_DIR ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/builtins)

  file(GLOB RT_BUILTINS_SRCS ${BUILTINS_DIR}/*.c)

  file(GLOB SRCSTMP ${BUILTINS_DIR}/${ARCH}/*.S)
  list(APPEND RT_BUILTINS_SRCS ${SRCSTMP})

  file(GLOB SRCSTMP ${BUILTINS_DIR}/${ARCH}/*.c)
  list(APPEND RT_BUILTINS_SRCS ${SRCSTMP})

  if(NOT CONFIG_LIB_COMPILER_RT_HAS_BFLOAT16)
    set(RT_BUILTINS_BFLOAT16_SRCS ${BUILTINS_DIR}/truncdfbf2.c
                                  ${BUILTINS_DIR}/truncsfbf2.c)
    list(REMOVE_ITEM RT_BUILTINS_SRCS ${RT_BUILTINS_BFLOAT16_SRCS})
  endif()

  set(x86_80_BIT_SOURCES
      ${BUILTINS_DIR}/divxc3.c
      ${BUILTINS_DIR}/extendxftf2.c
      ${BUILTINS_DIR}/fixxfdi.c
      ${BUILTINS_DIR}/fixxfti.c
      ${BUILTINS_DIR}/fixunsxfdi.c
      ${BUILTINS_DIR}/fixunsxfsi.c
      ${BUILTINS_DIR}/fixunsxfti.c
      ${BUILTINS_DIR}/floatdixf.c
      ${BUILTINS_DIR}/floattixf.c
      ${BUILTINS_DIR}/floatundixf.c
      ${BUILTINS_DIR}/floatuntixf.c
      ${BUILTINS_DIR}/mulxc3.c
      ${BUILTINS_DIR}/powixf2.c
      ${BUILTINS_DIR}/trunctfxf2.c)

  if(NOT CONFIG_ARCH_X86_64)
    list(REMOVE_ITEM RT_BUILTINS_SRCS ${x86_80_BIT_SOURCES})
  endif()

  # The Arm builtins ship hand-written VFP assembly whose FPU requirements the
  # globs above ignore, so assemble only the files the configured FPU supports
  # (upstream compiler-rt selects them conditionally).  chkstk.S and chkstk2.S
  # are Windows/MinGW-only stack probes; every *vfp.S needs a hardware FPU; and
  # the double-precision *df*vfp.S routines additionally need a double-precision
  # FPU.  Without this, single-precision-FPU targets such as Cortex-M33
  # (fpv5-sp-d16) fail with "selected FPU does not support instruction".  See
  # apache/nuttx#17386.
  if(CONFIG_ARCH_ARM)
    set(RT_BUILTINS_ARM_EXCLUDE ${BUILTINS_DIR}/arm/chkstk.S
                                ${BUILTINS_DIR}/arm/chkstk2.S)
    if(NOT CONFIG_ARCH_FPU)
      file(GLOB RT_BUILTINS_ARM_VFP ${BUILTINS_DIR}/arm/*vfp.S)
      list(APPEND RT_BUILTINS_ARM_EXCLUDE ${RT_BUILTINS_ARM_VFP})
    elseif(NOT CONFIG_ARCH_DPFPU)
      list(
        APPEND
        RT_BUILTINS_ARM_EXCLUDE
        ${BUILTINS_DIR}/arm/adddf3vfp.S
        ${BUILTINS_DIR}/arm/divdf3vfp.S
        ${BUILTINS_DIR}/arm/eqdf2vfp.S
        ${BUILTINS_DIR}/arm/extendsfdf2vfp.S
        ${BUILTINS_DIR}/arm/fixdfsivfp.S
        ${BUILTINS_DIR}/arm/fixunsdfsivfp.S
        ${BUILTINS_DIR}/arm/floatsidfvfp.S
        ${BUILTINS_DIR}/arm/floatunssidfvfp.S
        ${BUILTINS_DIR}/arm/gedf2vfp.S
        ${BUILTINS_DIR}/arm/gtdf2vfp.S
        ${BUILTINS_DIR}/arm/ledf2vfp.S
        ${BUILTINS_DIR}/arm/ltdf2vfp.S
        ${BUILTINS_DIR}/arm/muldf3vfp.S
        ${BUILTINS_DIR}/arm/nedf2vfp.S
        ${BUILTINS_DIR}/arm/subdf3vfp.S
        ${BUILTINS_DIR}/arm/truncdfsf2vfp.S
        ${BUILTINS_DIR}/arm/unorddf2vfp.S)
    endif()
    list(REMOVE_ITEM RT_BUILTINS_SRCS ${RT_BUILTINS_ARM_EXCLUDE})
  endif()

  if(NOT CONFIG_COVERAGE_NONE)
    target_compile_options(rt.builtins PRIVATE -fno-profile-instr-generate
                                               -fno-coverage-mapping)
  endif()

  target_sources(rt.builtins PRIVATE ${RT_BUILTINS_SRCS})

endif()

if(CONFIG_COVERAGE_COMPILER_RT)

  nuttx_add_system_library(rt.profile)

  target_include_directories(
    rt.profile PRIVATE ${INCDIR}
                       ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/profile)

  target_compile_options(rt.profile PRIVATE -fno-profile-instr-generate
                                            -fno-coverage-mapping)

  target_compile_options(rt.profile PRIVATE -DCOMPILER_RT_HAS_UNAME -Wno-undef
                                            -Wno-strict-prototypes -Wno-shadow)

  if(APPLE)
    target_compile_options(rt.profile PRIVATE -D__APPLE__)
  endif()

  set(SRCSTMP)
  set(RT_PROFILE_SRCS InstrProfilingPlatform.c)

  file(GLOB SRCSTMP ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/profile/*.c)
  list(APPEND RT_PROFILE_SRCS ${SRCSTMP})

  file(GLOB SRCSTMP ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/profile/*.cpp)
  list(APPEND RT_PROFILE_SRCS ${SRCSTMP})

  target_sources(rt.profile PRIVATE ${RT_PROFILE_SRCS})

elseif(CONFIG_COVERAGE_MINI AND CONFIG_ARCH_TOOLCHAIN_CLANG)
  nuttx_add_system_library(rt.miniprofile)

  target_include_directories(
    rt.miniprofile PRIVATE ${INCDIR}
                           ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/profile)

  target_compile_options(rt.miniprofile PRIVATE -fno-profile-instr-generate
                                                -fno-coverage-mapping)

  target_sources(rt.miniprofile PRIVATE ${CMAKE_CURRENT_LIST_DIR}/coverage.c)
endif()

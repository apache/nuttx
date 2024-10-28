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

if(CONFIG_BUILTIN_COMPILER_RT)

  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/compiler-rt)

    set(COMPILER_RT_VERSION ${CONFIG_COMPILER_RT_VERSION})

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

  nuttx_add_system_library(compiler-rt)

  list(APPEND INCDIR ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/include)
  list(APPEND INCDIR ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/builtins)

  # Arithmetic operations
  set(RT_BUILTIN_SRCS
      absvdi2.c
      absvsi2.c
      absvti2.c
      adddf3.c
      addtf3.c
      addvdi3.c
      addvsi3.c
      addvti3.c
      muldc3.c
      muldf3.c
      muldi3.c
      mulodi4.c
      mulosi4.c
      muloti4.c
      mulsc3.c
      mulsf3.c
      multc3.c
      multf3.c
      multi3.c
      mulvdi3.c
      mulvsi3.c
      mulvti3.c
      mulxc3.c
      negdf2.c
      negdi2.c
      negsf2.c
      negti2.c
      negvdi2.c
      negvsi2.c
      negvti2.c
      subdf3.c
      subsf3.c
      subtf3.c
      subvdi3.c
      subvsi3.c
      subvti3.c)

  # Floating point operations
  list(
    APPEND
    RT_BUILTIN_SRCS
    floatdidf.c
    floatdisf.c
    floatditf.c
    floatdixf.c
    floatsidf.c
    floatsisf.c
    floatsitf.c
    floattidf.c
    floattisf.c
    floattitf.c
    floattixf.c
    floatundidf.c
    floatundisf.c
    floatunditf.c
    floatundixf.c
    floatunsidf.c
    floatunsisf.c
    floatunsitf.c
    floatuntidf.c
    floatuntisf.c
    floatuntitf.c
    floatuntixf.c)

  # Conversion and expansion operations
  list(
    APPEND
    RT_BUILTIN_SRCS
    fixtfdi.c
    fixtfsi.c
    fixtfti.c
    fixunsdfdi.c
    fixunsdfsi.c
    fixunsdfti.c
    fixunssfdi.c
    fixunssfsi.c
    fixunssfti.c
    fixunstfdi.c
    fixunstfsi.c
    fixunstfti.c
    fixunsxfdi.c
    fixunsxfsi.c
    fixunsxfti.c
    fixxfdi.c
    fixxfti.c
    extenddftf2.c
    extendhfsf2.c
    extendhftf2.c
    extendsfdf2.c
    extendsftf2.c
    truncdfhf2.c
    truncdfsf2.c
    truncsfhf2.c
    trunctfdf2.c
    trunctfhf2.c
    trunctfsf2.c)

  # Bit manipulation operations
  list(
    APPEND
    RT_BUILTIN_SRCS
    ashldi3.c
    ashlti3.c
    ashrdi3.c
    ashrti3.c
    clzti2.c
    ctzdi2.c
    ctzsi2.c
    ctzti2.c
    lshrdi3.c
    lshrti3.c
    popcountdi2.c
    popcountsi2.c
    popcountti2.c
    paritydi2.c
    paritysi2.c
    parityti2.c)

  # Division and modulo operations
  list(
    APPEND
    RT_BUILTIN_SRCS
    divdc3.c
    divdf3.c
    divdi3.c
    divmoddi4.c
    divmodti4.c
    divsc3.c
    divsf3.c
    divtc3.c
    divtf3.c
    divti3.c
    divxc3.c
    moddi3.c
    modti3.c
    udivdi3.c
    udivmoddi4.c
    udivmodti4.c
    udivti3.c
    umoddi3.c
    umodti3.c)

  # Atomic operations
  list(
    APPEND
    RT_BUILTIN_SRCS
    atomic.c
    atomic_flag_clear.c
    atomic_flag_clear_explicit.c
    atomic_flag_test_and_set.c
    atomic_flag_test_and_set_explicit.c
    atomic_signal_fence.c
    atomic_thread_fence.c)

  # Other function related files
  list(
    APPEND
    RT_BUILTIN_SRCS
    apple_versioning.c
    clear_cache.c
    cmpdi2.c
    cmpti2.c
    comparedf2.c
    comparetf2.c
    cpu_model.c
    emutls.c
    enable_execute_stack.c
    eprintf.c
    fp_mode.c
    gcc_personality_v0.c
    int_util.c
    os_version_check.c
    trampoline_setup.c
    ucmpdi2.c
    ucmpti2.c
    powidf2.c
    powisf2.c
    powitf2.c
    powixf2.c)

  set(RT_BUILTIN_ARCH_SRCS)
  # ARM-specific assembly files
  if(CONFIG_ARCH_ARM)
    set(RT_BUILTIN_ARCH arm)
    list(
      APPEND
      RT_BUILTIN_ARCH_SRCS
      adddf3vfp.S
      addsf3.S
      addsf3vfp.S
      divmodsi4.S
      divsi3.S
      modsi3.S
      subdf3vfp.S
      subsf3vfp.S
      muldf3vfp.S
      mulsf3vfp.S
      negdf2vfp.S
      negsf2vfp.S)

    # Floating-Point Operations
    list(
      APPEND
      RT_BUILTIN_ARCH_SRCS
      comparesf2.S
      eqdf2vfp.S
      eqsf2vfp.S
      extendsfdf2vfp.S
      fixdfsivfp.S
      fixsfsivfp.S
      fixunsdfsivfp.S
      fixunssfsivfp.S
      floatsidfvfp.S
      floatsisfvfp.S
      floatunssidfvfp.S
      floatunssisfvfp.S
      gedf2vfp.S
      gesf2vfp.S
      gtdf2vfp.S
      gtsf2vfp.S
      ledf2vfp.S
      lesf2vfp.S
      ltdf2vfp.S
      ltsf2vfp.S
      nedf2vfp.S
      nesf2vfp.S
      truncdfsf2vfp.S
      unorddf2vfp.S
      unordsf2vfp.S)

    # Synchronization operations
    list(
      APPEND
      RT_BUILTIN_ARCH_SRCS
      sync_fetch_and_add_4.S
      sync_fetch_and_add_8.S
      sync_fetch_and_and_4.S
      sync_fetch_and_and_8.S
      sync_fetch_and_max_4.S
      sync_fetch_and_max_8.S
      sync_fetch_and_min_4.S
      sync_fetch_and_min_8.S
      sync_fetch_and_nand_4.S
      sync_fetch_and_nand_8.S
      sync_fetch_and_or_4.S
      sync_fetch_and_or_8.S
      sync_fetch_and_sub_4.S
      sync_fetch_and_sub_8.S
      sync_fetch_and_umax_4.S
      sync_fetch_and_umax_8.S
      sync_fetch_and_umin_4.S
      sync_fetch_and_umin_8.S
      sync_fetch_and_xor_4.S
      sync_fetch_and_xor_8.S
      sync_synchronize.S)

    # Memory operations
    list(
      APPEND
      RT_BUILTIN_ARCH_SRCS
      aeabi_memcmp.S
      aeabi_memcpy.S
      aeabi_memmove.S
      aeabi_memset.S
      restore_vfp_d8_d15_regs.S
      save_vfp_d8_d15_regs.S)

    # Division and Modulus Operations
    list(
      APPEND
      RT_BUILTIN_ARCH_SRCS
      aeabi_idivmod.S
      aeabi_ldivmod.S
      aeabi_uidivmod.S
      aeabi_uldivmod.S
      udivmodsi4.S
      udivsi3.S
      umodsi3.S)

    # Bitwise and Other Operations
    list(
      APPEND
      RT_BUILTIN_ARCH_SRCS
      bswapdi2.S
      bswapsi2.S
      chkstk.S
      clzdi2.S
      clzsi2.S)

    # Branch and Control Flow
    list(APPEND RT_BUILTIN_ARCH_SRCS switch16.S switch32.S switch8.S switchu8.S)

  else()

    # Other architectures
    list(
      APPEND
      RT_BUILTIN_SRCS
      addsf3.c
      divsi3.c
      modsi3.c
      bswapdi2.c
      bswapsi2.c
      clzdi2.c
      clzsi2.c
      divmodsi4.c
      udivmodsi4.c
      udivsi3.c
      umodsi3.c
      comparesf2.c)

  endif()

  if(CONFIG_ARCH_ARM64)
    set(ARCH "aarch64")
    list(APPEND RT_BUILTIN_ARCH_SRCS chkstk.S lse.S fp_mode.c)
  elseif(CONFIG_ARCH_RISCV)
    set(ARCH "riscv")
    list(
      APPEND
      RT_BUILTIN_ARCH_SRCS
      muldi3.S
      mulsi3.S
      restore.S
      save.S
      fp_mode.c)
  else()
    list(APPEND RT_BUILTIN_SRCS muldi3.c)
  endif()

  if(CONFIG_ARCH_X86_64)
    set(ARCH "x86_64")
    list(
      APPEND
      RT_BUILTIN_ARCH_SRCS
      chkstk2.S
      chkstk.S
      floatundidf.S
      floatundisf.S
      floatundixf.S
      floatdidf.c
      floatdisf.c
      floatdixf.c)
  else()
    list(APPEND RT_BUILTIN_SRCS floatundidf.c floatundisf.c floatundixf.c)
  endif()

  if(CONFIG_COMPILER_RT_PROFILE)

    target_include_directories(
      compiler-rt PRIVATE ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/profile)

    target_compile_options(
      compiler-rt
      PRIVATE -DCOMPILER_RT_HAS_UNAME
              -Wno-cleardeprecated-pragma
              -Wno-deprecated-pragma
              -Wno-incompatible-pointer-types
              -Wno-shadow
              -Wno-strict-prototypes
              -Wno-undef
              -Wno-unknown-warning-option)

    set(RT_PROFILE_SRCS
        GCDAProfiling.c
        InstrProfiling.c
        InstrProfilingBuffer.c
        InstrProfilingFile.c
        InstrProfilingInternal.c
        InstrProfilingMerge.c
        InstrProfilingMergeFile.c
        InstrProfilingNameVar.c
        InstrProfilingUtil.c
        InstrProfilingValue.c
        InstrProfilingVersionVar.c
        InstrProfilingWriter.c
        InstrProfilingRuntime.cpp)

    foreach(src ${RT_PROFILE_SRCS})
      string(PREPEND src ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/profile/)
      list(APPEND COMPILER_RT_SRCS ${src})
    endforeach()

    list(APPEND COMPILER_RT_SRCS InstrProfilingPlatform.c)

  endif()

  list(APPEND INCDIR
       ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/builtins/${RT_BUILTIN_ARCH})

  foreach(src ${RT_BUILTIN_SRCS})
    string(PREPEND src ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/builtins/)
    list(APPEND COMPILER_RT_SRCS ${src})
  endforeach()

  foreach(src ${RT_BUILTIN_ARCH_SRCS})
    string(
      PREPEND src
      ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/builtins/${RT_BUILTIN_ARCH}/)
    list(APPEND COMPILER_RT_SRCS ${src})
  endforeach()

  # Add the sources to the target
  target_sources(compiler-rt PRIVATE ${COMPILER_RT_SRCS})

  target_compile_options(
    compiler-rt
    PRIVATE -Wno-shift-count-negative
            -Wno-constant-conversion
            -Wshift-count-overflow
            -Wno-undef
            -Wno-incompatible-pointer-types
            -Wno-visibility
            -Wno-macro-redefined)

  target_include_directories(compiler-rt PRIVATE ${INCDIR})
endif()

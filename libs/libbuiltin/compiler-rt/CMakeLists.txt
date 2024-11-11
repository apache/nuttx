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

    nuttx_add_system_library(rt.buitlins)

    target_include_directories(
      rt.buitlins PRIVATE ${INCDIR}
                          ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/builtins)

    target_compile_options(rt.buitlins PRIVATE -Wno-undef -Wno-macro-redefined)

    set(SRCSTMP)
    set(RT_BUILTINS_SRCS)
    file(GLOB RT_BUILTINS_SRCS
         ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/builtins/*.c)

    file(GLOB SRCSTMP
         ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/builtins/${ARCH}/*.S)
    list(APPEND RT_BUILTINS_SRCS ${SRCSTMP})

    file(GLOB SRCSTMP
         ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/builtins/${ARCH}/*.c)
    list(APPEND RT_BUILTINS_SRCS ${SRCSTMP})

    if(NOT CONFIG_LIB_COMPILER_RT_HAS_BFLOAT16)
      set(RT_BUILTINS_BFLOAT16_SRCS
          ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/builtins/truncdfbf2.c
          ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/builtins/truncsfbf2.c)
      list(REMOVE_ITEM RT_BUILTINS_SRCS ${RT_BUILTINS_BFLOAT16_SRCS})
    endif()

    target_sources(rt.buitlins PRIVATE ${RT_BUILTINS_SRCS})

  endif()

  if(CONFIG_COVERAGE_COMPILER_RT)

    nuttx_add_system_library(rt.profile)

    target_include_directories(
      rt.profile PRIVATE ${INCDIR}
                         ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/profile)

    target_compile_options(
      rt.profile PRIVATE -DCOMPILER_RT_HAS_UNAME -Wno-undef
                         -Wno-strict-prototypes -Wno-shadow)

    set(SRCSTMP)
    set(RT_PROFILE_SRCS InstrProfilingPlatform.c)

    file(GLOB SRCSTMP ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/profile/*.c)
    list(APPEND RT_PROFILE_SRCS ${SRCSTMP})

    file(GLOB SRCSTMP ${CMAKE_CURRENT_LIST_DIR}/compiler-rt/lib/profile/*.cpp)
    list(APPEND RT_PROFILE_SRCS ${SRCSTMP})

    target_sources(rt.profile PRIVATE ${RT_PROFILE_SRCS})

  endif()

endif()

# ##############################################################################
# libs/libxx/libcxxabi/CMakeLists.txt
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
if(CONFIG_LIBCXXABI)
  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/libcxxabi)

    set(LIBCXXABI_VERSION ${CONFIG_LIBCXXABI_VERSION})

    FetchContent_Declare(
      libcxxabi
      DOWNLOAD_NAME "libcxxabi-${LIBCXXABI_VERSION}.src.tar.xz"
      DOWNLOAD_DIR ${CMAKE_CURRENT_LIST_DIR}
      URL "https://github.com/llvm/llvm-project/releases/download/llvmorg-${LIBCXXABI_VERSION}/libcxxabi-${LIBCXXABI_VERSION}.src.tar.xz"
          SOURCE_DIR
          ${CMAKE_CURRENT_LIST_DIR}/libcxxabi
          BINARY_DIR
          ${CMAKE_BINARY_DIR}/libs/libc/libcxxabi
          CONFIGURE_COMMAND
          ""
          BUILD_COMMAND
          ""
          INSTALL_COMMAND
          ""
          TEST_COMMAND
          ""
      PATCH_COMMAND ""
      DOWNLOAD_NO_PROGRESS true
      TIMEOUT 30)

    FetchContent_GetProperties(libcxxabi)

    if(NOT libcxxabi_POPULATED)
      FetchContent_Populate(libcxxabi)
    endif()
  endif()

  nuttx_create_symlink(${CMAKE_CURRENT_LIST_DIR}/libcxxabi/include
                       ${CMAKE_BINARY_DIR}/include/libcxxabi)

  set_property(
    TARGET nuttx
    APPEND
    PROPERTY NUTTX_CXX_INCLUDE_DIRECTORIES
             ${CMAKE_BINARY_DIR}/include/libcxxabi)

  nuttx_add_system_library(libcxxabi)

  # C++ABI files
  list(
    APPEND
    SRCS
    cxa_aux_runtime.cpp
    cxa_default_handlers.cpp
    cxa_demangle.cpp
    cxa_exception_storage.cpp
    cxa_guard.cpp
    cxa_handlers.cpp
    cxa_thread_atexit.cpp
    cxa_vector.cpp
    cxa_virtual.cpp)
  add_compile_definitions(_LIBCPP_BUILDING_LIBRARY)
  if(CONFIG_LIBSUPCXX_TOOLCHAIN)
    add_compile_definitions(__GLIBCXX__)
  endif()

  if(CONFIG_LIBSUPCXX)
    add_compile_definitions(__GLIBCXX__)
  endif()
  # C++ STL files
  list(APPEND SRCS stdlib_exception.cpp stdlib_new_delete.cpp
       stdlib_stdexcept.cpp stdlib_typeinfo.cpp)

  # Internal files
  list(APPEND SRCS abort_message.cpp fallback_malloc.cpp private_typeinfo.cpp)

  if(CONFIG_CXX_EXCEPTION)
    list(APPEND SRCS cxa_exception.cpp cxa_personality.cpp)
  endif()

  if(CONFIG_LIBCXXABI)
    add_compile_definitions(LIBCXX_BUILDING_LIBCXXABI)
  endif()

  foreach(src ${SRCS})
    string(PREPEND src libcxxabi/src/)
    list(APPEND TARGET_SRCS ${src})
  endforeach()

  # RTTI is required for building the libcxxabi library
  target_compile_options(libcxxabi PRIVATE -frtti)

  if(CONFIG_SIM_UBSAN OR CONFIG_MM_UBSAN)
    target_compile_options(libcxxabi PRIVATE -fno-sanitize=vptr)
  endif()

  # Fix compilation error on ARM32:libcxxabi/src/cxa_personality.cpp:594:22:
  # error: '_URC_FATAL_PHASE1_ERROR' was not declared in this scope 594 |
  # results.reason = _URC_FATAL_PHASE1_ERROR;
  if(CONFIG_ARCH_ARM)
    target_compile_definitions(libcxxabi
                               PRIVATE _URC_FATAL_PHASE2_ERROR=_URC_FAILURE)
    target_compile_definitions(libcxxabi
                               PRIVATE _URC_FATAL_PHASE1_ERROR=_URC_FAILURE)
  endif()

  target_compile_definitions(libcxxabi
                             PRIVATE LIBCXXABI_NON_DEMANGLING_TERMINATE)

  target_sources(libcxxabi PRIVATE ${TARGET_SRCS})
  target_include_directories(
    libcxxabi BEFORE PRIVATE ${NUTTX_DIR}/libs/libxx/libcxx/libcxx/src
                             ${CMAKE_CURRENT_LIST_DIR}/libcxxabi/include)
endif()

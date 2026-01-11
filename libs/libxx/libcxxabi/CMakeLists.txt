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
      PATCH_COMMAND
        patch -p1 -d ${CMAKE_CURRENT_LIST_DIR}/libcxxabi <
        ${CMAKE_CURRENT_LIST_DIR}/0001-libcxxabi-Fix-build-warnings-generated-by-CMake-comp.patch
        && patch -p1 -d ${CMAKE_CURRENT_LIST_DIR}/libcxxabi <
        ${CMAKE_CURRENT_LIST_DIR}/0002-libcxxabi-fix-compilation-errors.patch &&
        patch -p0 -d ${CMAKE_CURRENT_LIST_DIR}/libcxxabi <
        ${CMAKE_CURRENT_LIST_DIR}/0003-libcxxabi-fix-exception-handler-init_21.1.8.patch
        && patch -p1 -d ${CMAKE_CURRENT_LIST_DIR}/libcxxabi <
        ${CMAKE_CURRENT_LIST_DIR}/0004-libcxxabi-disable-threading-in-cxa-guard_21.1.8.patch
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
    cxa_guard.cpp
    cxa_handlers.cpp
    cxa_vector.cpp
    cxa_virtual.cpp)
  # Excluded: cxa_exception_storage.cpp (uses thread-local storage
  # __aeabi_read_tp) Excluded threading-dependent files: cxa_thread_atexit.cpp
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

  # Internal files and stub implementations (no threading, no TLS)
  list(APPEND SRCS abort_message.cpp private_typeinfo.cpp)

  # Add stub files from libs/libxx/libcxxabi/src/ (not from libcxxabi/src/)
  list(APPEND STUB_SRCS
       ${CMAKE_CURRENT_LIST_DIR}/src/cxa_exception_storage_stub.cpp
       ${CMAKE_CURRENT_LIST_DIR}/src/fallback_malloc_stub.cpp)

  # Exclude threading-dependent files when _LIBCPP_HAS_NO_THREADS is defined
  # These files use __libcpp_mutex_t, __libcpp_condvar_t, __libcpp_tls_key which
  # are not available without threading list(APPEND SRCS cxa_guard.cpp
  # cxa_thread_atexit.cpp)

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

  # Fix ARM unwinder enum type conversion warnings without disabling all type
  # safety
  if(CONFIG_ARCH_ARM)
    target_compile_options(libcxxabi PRIVATE -Wno-error=conversion
                                             -Wno-error=enum-conversion)
  endif()

  # Disable threading support to match libcxx configuration Note:
  # _LIBCPP_HAS_NO_THREADS is already defined in __config_site
  target_compile_definitions(libcxxabi PRIVATE _LIBCXXABI_HAS_NO_THREADS)

  if(CONFIG_SIM_UBSAN OR CONFIG_MM_UBSAN)
    target_compile_options(libcxxabi PRIVATE -fno-sanitize=vptr)
  endif()

  # Fix compilation error on ARM32:libcxxabi/src/cxa_personality.cpp:594:22:
  # error: '_URC_FATAL_PHASE1_ERROR' was not declared in this scope 594 |
  # results.reason = _URC_FATAL_PHASE1_ERROR;
  if(CONFIG_ARCH_ARM)
    # Enable bare-metal mode for correct TARGET2 relocation handling
    target_compile_definitions(libcxxabi PRIVATE LIBCXXABI_BAREMETAL)
    target_compile_definitions(libcxxabi
                               PRIVATE _URC_FATAL_PHASE2_ERROR=_URC_FAILURE)
    target_compile_definitions(libcxxabi
                               PRIVATE _URC_FATAL_PHASE1_ERROR=_URC_FAILURE)
  endif()

  target_compile_definitions(libcxxabi
                             PRIVATE LIBCXXABI_NON_DEMANGLING_TERMINATE)

  target_sources(libcxxabi PRIVATE ${TARGET_SRCS} ${STUB_SRCS})
  target_include_directories(
    libcxxabi BEFORE PRIVATE ${NUTTX_DIR}/libs/libxx/libcxx/libcxx/src
                             ${CMAKE_CURRENT_LIST_DIR}/libcxxabi/include)
endif()

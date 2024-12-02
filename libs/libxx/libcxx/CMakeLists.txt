# ##############################################################################
# libs/libxx/libcxx/CMakeLists.txt
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

if(CONFIG_LIBCXX)
  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/libcxx)

    set(LIBCXX_VERSION ${CONFIG_LIBCXX_VERSION})

    FetchContent_Declare(
      libcxx
      DOWNLOAD_NAME "libcxx-${LIBCXX_VERSION}.src.tar.xz"
      DOWNLOAD_DIR ${CMAKE_CURRENT_LIST_DIR}
      URL "https://github.com/llvm/llvm-project/releases/download/llvmorg-${LIBCXX_VERSION}/libcxx-${LIBCXX_VERSION}.src.tar.xz"
          SOURCE_DIR
          ${CMAKE_CURRENT_LIST_DIR}/libcxx
          BINARY_DIR
          ${CMAKE_BINARY_DIR}/libs/libc/libcxx
          CONFIGURE_COMMAND
          ""
          BUILD_COMMAND
          ""
          INSTALL_COMMAND
          ""
          TEST_COMMAND
          ""
      PATCH_COMMAND
        patch -p1 -d ${CMAKE_CURRENT_LIST_DIR}/libcxx <
        ${CMAKE_CURRENT_LIST_DIR}/0001_fix_stdatomic_h_miss_typedef.patch &&
        patch -p3 -d ${CMAKE_CURRENT_LIST_DIR}/libcxx <
        ${CMAKE_CURRENT_LIST_DIR}/mbstate_t.patch && patch -p1 -d
        ${CMAKE_CURRENT_LIST_DIR}/libcxx <
        ${CMAKE_CURRENT_LIST_DIR}/0001-libcxx-remove-mach-time-h.patch && patch
        -p1 -d ${CMAKE_CURRENT_LIST_DIR}/libcxx <
        ${CMAKE_CURRENT_LIST_DIR}/0001-libcxx-fix-ld-errors.patch && patch -p1
        -d ${CMAKE_CURRENT_LIST_DIR}/libcxx <
        ${CMAKE_CURRENT_LIST_DIR}/0001-Fix-build-error-about-__GLIBC__.patch
      DOWNLOAD_NO_PROGRESS true
      TIMEOUT 30)

    FetchContent_GetProperties(libcxx)

    if(NOT libcxx_POPULATED)
      FetchContent_Populate(libcxx)
    endif()

  endif()

  nuttx_create_symlink(${CMAKE_CURRENT_LIST_DIR}/libcxx/include
                       ${CMAKE_BINARY_DIR}/include/libcxx)

  configure_file(${CMAKE_CURRENT_LIST_DIR}/../__config_site
                 ${CMAKE_BINARY_DIR}/include/libcxx/__config_site COPYONLY)

  set_property(
    TARGET nuttx
    APPEND
    PROPERTY NUTTX_CXX_INCLUDE_DIRECTORIES ${CMAKE_BINARY_DIR}/include/libcxx)

  add_compile_definitions(_LIBCPP_BUILDING_LIBRARY)
  if(CONFIG_LIBSUPCXX_TOOLCHAIN)
    add_compile_definitions(__GLIBCXX__)
  endif()

  if(CONFIG_LIBSUPCXX)
    add_compile_definitions(__GLIBCXX__)
  endif()

  if(CONFIG_LIBCXXABI)
    add_compile_definitions(LIBCXX_BUILDING_LIBCXXABI)
  endif()

  file(GLOB SRCS ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/*.cpp)
  file(GLOB SRCSTMP ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/experimental/*.cpp)
  list(APPEND SRCS ${SRCSTMP})
  file(GLOB SRCSTMP ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/filesystem/*.cpp)
  list(APPEND SRCS ${SRCSTMP})
  file(GLOB SRCSTMP ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/ryu/*.cpp)
  list(APPEND SRCS ${SRCSTMP})

  if(NOT CONFIG_CXX_LOCALIZATION)
    file(
      GLOB
      SRCSTMP
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/ios.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/ios.instantiations.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/iostream.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/locale.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/regex.cpp
      ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/strstream.cpp)
    list(REMOVE_ITEM SRCS ${SRCSTMP})
  endif()

  set(FLAGS -Wno-attributes -Wno-deprecated-declarations -Wno-shadow
            -Wno-sign-compare -Wno-cpp)

  if(GCCVER GREATER_EQUAL 12)
    list(APPEND FLAGS -Wno-maybe-uninitialized -Wno-alloc-size-larger-than)
  endif()

  nuttx_add_system_library(libcxx)
  target_sources(libcxx PRIVATE ${SRCS})
  target_compile_options(libcxx PRIVATE ${FLAGS})
  target_include_directories(libcxx BEFORE
                             PRIVATE ${CMAKE_CURRENT_LIST_DIR}/libcxx/src)
endif()

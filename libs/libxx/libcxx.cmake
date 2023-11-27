# ##############################################################################
# libs/libxx/libcxx.cmake
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

if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/libcxx)

  set(LIBCXX_VERSION ${CONFIG_LIBCXX_VERSION})

  # cmake-format: off
  set(LIBCXX_PATCH_COMMAND
  patch -p1 -d ${CMAKE_CURRENT_LIST_DIR}/libcxx < ${CMAKE_CURRENT_LIST_DIR}/0001_fix_stdatomic_h_miss_typedef.patch)
  # cmake-format: on
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
    PATCH_COMMAND ${LIBCXX_PATCH_COMMAND}
    DOWNLOAD_NO_PROGRESS true
    TIMEOUT 30)

  FetchContent_GetProperties(libcxx)

  if(NOT libcxx_POPULATED)
    FetchContent_Populate(libcxx)
  endif()

  execute_process(
    COMMAND
      sh -c
      "ln -s ${CMAKE_CURRENT_LIST_DIR}/libcxx/include ${NUTTX_DIR}/include/libcxx"
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
  execute_process(
    COMMAND
      sh -c
      "cp ${CMAKE_CURRENT_LIST_DIR}/__config_site ${NUTTX_DIR}/include/libcxx/__config_site"
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR})
endif()

find_program(GN_EXECUTABLEXX gn REQUIRED)
message("GN_EXECUTABLEXX ${GN_EXECUTABLEXX}")

set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR}/libcxx/include)

set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_LIST_DIR}/libcxx/src)

add_compile_definitions(_LIBCPP_BUILDING_LIBRARY)
if(CONFIG_LIBSUPCXX)
  add_compile_definitions(__GLIBCXX__)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SRCS)
set(SRCSTMP)

file(GLOB SRCS ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/*.cpp)
file(GLOB SRCSTMP ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/experimental/*.cpp)
list(APPEND SRCS ${SRCSTMP})
file(GLOB SRCSTMP ${CMAKE_CURRENT_LIST_DIR}/libcxx/src/filesystem/*.cpp)
list(APPEND SRCS ${SRCSTMP})

set_source_files_properties(libcxx/src/barrier.cpp PROPERTIES COMPILE_FLAGS
                                                              -Wno-shadow)
set_source_files_properties(libcxx/src/locale.cpp PROPERTIES COMPILE_FLAGS
                                                             -Wno-shadow)
set_source_files_properties(libcxx/src/filesystem/directory_iterator.cpp
                            PROPERTIES COMPILE_FLAGS -Wno-shadow)
set_source_files_properties(libcxx/src/filesystem/operations.cpp
                            PROPERTIES COMPILE_FLAGS -Wno-shadow)

nuttx_add_system_library(libcxx)
target_sources(libcxx PRIVATE ${SRCS})

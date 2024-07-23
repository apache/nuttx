# ##############################################################################
# libs/libxx/uClibc++.cmake
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

set(UCLIBCXX_DIR ${CMAKE_CURRENT_LIST_DIR}/uClibc++)

if(NOT EXISTS ${UCLIBCXX_DIR})

  set(UCLIBCXX_VERSION 0.2.5)

  FetchContent_Declare(
    uClibc++
    DOWNLOAD_NAME "uClibc++-${UCLIBCXX_VERSION}.tar.bz2"
    DOWNLOAD_DIR ${CMAKE_CURRENT_LIST_DIR}
    URL "https://git.busybox.net/uClibc++/snapshot/uClibc++-${UCLIBCXX_VERSION}.tar.bz2"
        SOURCE_DIR
        ${CMAKE_CURRENT_LIST_DIR}/uClibc++
        BINARY_DIR
        ${CMAKE_BINARY_DIR}/libs/libc/uClibc++
        CONFIGURE_COMMAND
        ""
        BUILD_COMMAND
        ""
        INSTALL_COMMAND
        ""
        TEST_COMMAND
        ""
    PATCH_COMMAND
      patch -p1 -d ${CMAKE_CURRENT_LIST_DIR}/uClibc++ <
      ${CMAKE_CURRENT_LIST_DIR}/0001-uClibcxx-basic_definitions-fix-GCC-specific-definiti.patch
      && patch -p1 -d ${CMAKE_CURRENT_LIST_DIR}/uClibc++ <
      ${CMAKE_CURRENT_LIST_DIR}/0001-uclibxx-use-overload-constructor-of-filebuf-ostream.patch
    DOWNLOAD_NO_PROGRESS true
    TIMEOUT 30)

  FetchContent_GetProperties(uClibc++)

  if(NOT uClibc++_POPULATED)
    FetchContent_Populate(uClibc++)
  endif()

endif()

nuttx_create_symlink(${CMAKE_CURRENT_LIST_DIR}/uClibc++/include
                     ${CMAKE_BINARY_DIR}/include/uClibc++)

configure_file(
  ${CMAKE_CURRENT_LIST_DIR}/system_configuration.h
  ${CMAKE_BINARY_DIR}/include/uClibc++/system_configuration.h COPYONLY)

set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_CXX_INCLUDE_DIRECTORIES ${CMAKE_BINARY_DIR}/include/uClibc++)

set(SRCS
    algorithm.cpp
    associative_base.cpp
    bitset.cpp
    char_traits.cpp
    complex.cpp
    deque.cpp
    exception.cpp
    fstream.cpp
    func_exception.cpp
    iomanip.cpp
    ios.cpp
    iostream.cpp
    istream.cpp
    iterator.cpp
    limits.cpp
    list.cpp
    locale.cpp
    map.cpp
    numeric.cpp
    ostream.cpp
    queue.cpp
    set.cpp
    sstream.cpp
    stack.cpp
    stdexcept.cpp
    streambuf.cpp
    string.cpp
    utility.cpp
    valarray.cpp
    vector.cpp)

set(TARGET_SRCS)

foreach(src ${SRCS})
  string(PREPEND src uClibc++/src/)
  list(APPEND TARGET_SRCS ${src})
endforeach()

nuttx_add_system_library(uClibc++)
target_sources(uClibc++ PRIVATE ${TARGET_SRCS})

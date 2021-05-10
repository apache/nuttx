############################################################################
# libs/libxx/libcxx.defs
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
###########################################################################

set(LIBCXX_VERSION 11.0.0)

FetchContent_Declare(libcxx
  URL https://github.com/llvm/llvm-project/releases/download/llvmorg-${LIBCXX_VERSION}/libcxx-${LIBCXX_VERSION}.src.tar.xz
  PATCH_COMMAND patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0001-libc-Fix-a-few-warnings.patch &&
    patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0001-libc-NFC-Fix-several-GCC-warnings-in-the-test-suite.patch &&
    patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0001-libc-Fix-tests-failing-with-Clang-after-removing-GCC.patch &&
    patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0001-libcxx-Check-_LIBCPP_PROVIDES_DEFAULT_RUNE_TABLE-fir.patch &&
    patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0001-libcxx-Port-to-NuttX-https-nuttx.apache.org-RTOS.patch
)
FetchContent_Populate(libcxx)
FetchContent_GetProperties(libcxx SOURCE_DIR LIBCXX_SOURCE_DIR)

file(GLOB SRCS LIST_DIRECTORIES false CONFIGURE_DEPENDS
  ${LIBCXX_SOURCE_DIR}/src/*.cpp ${LIBCXX_SOURCE_DIR}/src/experimental/*.cpp ${LIBCXX_SOURCE_DIR}/src/filesystem/*.cpp)
target_sources(xx PRIVATE ${SRCS})

set_property(TARGET nuttx APPEND PROPERTY NUTTX_DEFINITIONS
  $<$<COMPILE_LANGUAGE:CXX>:__GLIBCXX__>
  $<$<COMPILE_LANGUAGE:CXX>:_LIBCPP_BUILDING_LIBRARY>)

# Workaround the following warning with "c++ (Ubuntu 9.3.0-10ubuntu2) 9.3.0"
#
# libcxx/src/barrier.cpp: In constructor 'std::__1::__barrier_algorithm_base::__barrier_algorithm_base(ptrdiff_t&)':
# libcxx/src/barrier.cpp:35:9: warning: declaration of '__expected' shadows a member of 'std::__1::__barrier_algorithm_base' [-Wshadow]
#    35 |         : __expected(__expected)
#       |         ^
# libcxx/src/barrier.cpp:29:24: note: shadowed declaration is here
#    29 |     ptrdiff_t&         __expected;
#       |                        ^~~~~~~~~~

set_property(SOURCE ${SRCS} APPEND PROPERTY COMPILE_OPTIONS -Wno-shadow)

execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink ${LIBCXX_SOURCE_DIR}/include ${CMAKE_BINARY_DIR}/include_cxx
)


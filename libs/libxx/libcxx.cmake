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
  URL https://github.com/llvm/llvm-project/releases/download/llvmorg-${VERSION}/libcxx-${VERSION}.src.tar.xz
  PATCH_COMMAND -p0 < libcxx-fixes.patch
)
FetchContent_Populate(libcxx)


file(GLOB SRCS LIST_DIRECTORIES false CONFIGURE_DEPENDS
  libcxx/src/*.cpp libcxx/src/experimental/*.cpp libcxx/src/filesystem/*.cpp)
target_sources(xx PRIVATE ${SRCS})

set_property(GLOBAL APPEND PROPERTY NUTTX_DEFINITIONS
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

set_property(SOURCE libcxx/src/barrier.cpp APPEND PROPERTY COMPILE_OPTIONS -Wno-shadow)
set_property(SOURCE libcxx/src/locale.cpp APPEND PROPERTY COMPILE_OPTIONS -Wno-shadow)

set_property(SOURCE libcxx/src/filesystem/directory_iterator.cpp APPEND PROPERTY COMPILE_OPTIONS -Wno-shadow)
set_property(SOURCE libcxx/src/filesystem/operations.cpp APPEND PROPERTY COMPILE_OPTIONS -Wno-shadow)

# ##############################################################################
# libs/libxx/libcxxmini.cmake
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

nuttx_add_system_library(libcxxmini)

set_source_files_properties(
  libcxxmini/libxx_new.cxx PROPERTIES COMPILE_FLAGS -Wno-missing-exception-spec)
set_source_files_properties(
  libcxxmini/libxx_newa.cxx PROPERTIES COMPILE_FLAGS
                                       -Wno-missing-exception-spec)

target_sources(
  libcxxmini
  PRIVATE libcxxmini/libxx_cxa_guard.cxx
          libcxxmini/libxx_cxapurevirtual.cxx
          libcxxmini/libxx_delete.cxx
          libcxxmini/libxx_delete_sized.cxx
          libcxxmini/libxx_deletea.cxx
          libcxxmini/libxx_deletea_sized.cxx
          libcxxmini/libxx_new.cxx
          libcxxmini/libxx_newa.cxx)

# Why c++14? * libcxx seems to require c++11. * The compiler defaults varies:
# clang/macOS (from xcode): 199711L gcc/ubuntu:               201402L * There is
# a precedent to use c++14. (boards/arm/stm32l4/nucleo-l476rg/scripts/Make.defs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_CXX_INCLUDE_DIRECTORIES ${NUTTX_DIR}/include/cxx)

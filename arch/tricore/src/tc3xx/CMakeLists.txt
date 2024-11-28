# ##############################################################################
# arch/tricore/src/tc3xx/CMakeLists.txt
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

set(SRCS tc3xx_timerisr.c tc3xx_serial.c)

target_sources(arch PRIVATE ${SRCS})

if(CONFIG_ARCH_CHIP_TC397)

  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/tc397)
    FetchContent_Declare(
      tc397
      URL https://github.com/anchao/tc397_sdk/archive/refs/heads/master.tar.gz
          SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/tc397 BINARY_DIR
          ${CMAKE_BINARY_DIR}/arch/${CONFIG_ARCH}/src/${CONFIG_ARCH_CHIP}/tc397)
    FetchContent_GetProperties(tc397)
    if(NOT tc397_POPULATED)
      FetchContent_Populate(tc397)
    endif()
  endif()

  add_subdirectory(tc397)

  if(CONFIG_TRICORE_TOOLCHAIN_TASKING)
    nuttx_add_kernel_library(c_fpu)
    target_sources(c_fpu PRIVATE tc3xx_libc.c)
    target_link_options(
      nuttx PRIVATE
      --library-directory=${CMAKE_BINARY_DIR}/arch/tricore/src/exclude_chip)
  endif()

endif()

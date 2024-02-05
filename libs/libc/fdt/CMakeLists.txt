# ##############################################################################
# libs/libc/fdt/CMakeLists.txt
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
if(CONFIG_LIBC_FDT)

  set(FDT_DIR ${CMAKE_CURRENT_LIST_DIR}/dtc)
  if(NOT EXISTS ${FDT_DIR})
    set(VERSION ${CONFIG_LIBC_FDT_DTC_VERSION})

    FetchContent_Declare(
      dtc
      URL https://github.com/dgibson/dtc/archive/v${VERSION}.zip SOURCE_DIR
          ${CMAKE_CURRENT_LIST_DIR}/dtc BINARY_DIR
          ${CMAKE_BINARY_DIR}/libs/libc/fdt/dtc)
    FetchContent_GetProperties(dtc)
    if(NOT dtc_POPULATED)
      FetchContent_Populate(dtc)
    endif()
  endif()

  set(SRCS
      fdt.c
      fdt_ro.c
      fdt_wip.c
      fdt_sw.c
      fdt_rw.c
      fdt_strerror.c
      fdt_empty_tree.c
      fdt_addresses.c
      fdt_overlay.c
      fdt_check.c)
  list(TRANSFORM SRCS PREPEND ${FDT_DIR}/libfdt/)

  target_sources(c PRIVATE ${SRCS})

  target_include_directories(c PRIVATE ${FDT_DIR}/libfdt)
endif()

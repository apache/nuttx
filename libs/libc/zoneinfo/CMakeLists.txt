# ##############################################################################
# libs/libc/zoneinfo/CMakeLists.txt
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
if(CONFIG_LIB_ZONEINFO_ROMFS)
  FetchContent_Declare(tzcode URL http://ftp.iana.org/tz/tzcode-latest.tar.gz)
  FetchContent_Declare(tzdata URL http://ftp.iana.org/tz/tzdata-latest.tar.gz)
  FetchContent_Populate(tzcode)
  FetchContent_Populate(tzdata)

  file(COPY ${tzcode_SOURCE_DIR}/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/tz)
  file(COPY ${tzdata_SOURCE_DIR}/ DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/tz)

  execute_process(COMMAND make -C ${CMAKE_CURRENT_BINARY_DIR}/tz install
                          TOPDIR=${CMAKE_CURRENT_BINARY_DIR}/tzbin)

  nuttx_add_romfs(
    NAME
    zoneinfo
    PATH
    ${CMAKE_CURRENT_BINARY_DIR}/tzbin/usr/share/zoneinfo
    HEADER
    PREFIX
    zoneinfo
    NONCONST)

  set_property(SOURCE romfs_zoneinfo.h PROPERTY GENERATED)

  nuttx_add_aux_library(zoneinfo tzromfs.c)
  target_include_directories(zoneinfo PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
  target_link_libraries(c PRIVATE zoneinfo)
endif()

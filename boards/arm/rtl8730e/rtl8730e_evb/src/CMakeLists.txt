# ##############################################################################
# boards/arm/rtl8730e/rtl8730e_evb/src/CMakeLists.txt
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
set(SRCS rtl8730e_boardinit.c rtl8730e_bringup.c)

if(CONFIG_AMEBA_GPIO)
  list(APPEND SRCS rtl8730e_gpio.c)
  target_include_directories(board
                             PRIVATE ${NUTTX_DIR}/arch/arm/src/common/ameba)
endif()

if(CONFIG_LIBC_FDT)
  target_include_directories(board
                             PRIVATE ${NUTTX_DIR}/libs/libc/fdt/dtc/libfdt)
endif()

target_sources(board PRIVATE ${SRCS})

if(CONFIG_ETC_ROMFS AND CONFIG_SYSTEM_NXINIT)
  nuttx_add_romfs(
    NAME
    etc
    MOUNTPOINT
    etc
    RCSRCS
    etc/init.d/init.rc
    PATH
    ${CMAKE_CURRENT_BINARY_DIR}/etc)

  target_link_libraries(board PRIVATE romfs_etc)
endif()

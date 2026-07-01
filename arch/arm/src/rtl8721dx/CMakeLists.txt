# ##############################################################################
# arch/arm/src/rtl8721dx/CMakeLists.txt
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

# IC-specific register drivers + IPC wiring live here; IC-agnostic glue is
# shared from arch/arm/src/common/ameba.
set(SRCS ameba_allocateheap.c ameba_irq.c ameba_loguart.c ameba_start.c
         ameba_timerisr.c)

set(AMEBA_COMMON ${CMAKE_CURRENT_LIST_DIR}/../common/ameba)

list(APPEND SRCS ${AMEBA_COMMON}/ameba_os_wrap.c)

if(CONFIG_RTL8721DX_WIFI)
  list(APPEND SRCS ameba_wifi_init.c ${AMEBA_COMMON}/ameba_kv.c)
  if(CONFIG_NET)
    list(APPEND SRCS ${AMEBA_COMMON}/ameba_wlan.c)
  endif()
endif()

if(CONFIG_RTL8721DX_FLASH_FS)
  list(APPEND SRCS ${AMEBA_COMMON}/ameba_flash_mtd.c)
endif()

target_include_directories(arch PRIVATE ${AMEBA_COMMON})
target_sources(arch PRIVATE ${SRCS})

# ##############################################################################
# arch/arm/src/rtl8730e/CMakeLists.txt
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

set(SRCS rtl8730e_boot.c rtl8730e_serial.c rtl8730e_irq.c rtl8730e_timer.c)

if(CONFIG_ARCH_ARMV7A)
  list(APPEND SRCS rtl8730e_pgalloc.c rtl8730e_memorymap.c)
endif()

if(CONFIG_SMP)
  list(APPEND SRCS rtl8730e_cpuboot.c)
endif()

if(CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS rtl8730e_idle.c)
endif()

if(CONFIG_BUILD_KERNEL)
  list(APPEND SRCS rtl8730e_pgalloc.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS rtl8730e_userspace.c rtl8730e_allocateheap.c)
endif()

target_sources(arch PRIVATE ${SRCS})

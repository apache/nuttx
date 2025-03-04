# ##############################################################################
# arch/risc-v/src/k230/CMakeLists.txt
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

set(SRCS k230_head.S k230_start.c k230_irq.c k230_irq_dispatch.c)

list(APPEND SRCS k230_timerisr.c k230_allocateheap.c k230_hart.c)

if(CONFIG_BUILD_KERNEL)
  list(APPEND SRCS k230_mm_init.c)
elseif(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS k230_userspace.c)
endif()

if(CONFIG_MM_PGALLOC)
  list(APPEND SRCS k230_pgalloc.c)
endif()

if(CONFIG_K230_IPI)
  list(APPEND SRCS k230_ipi.c)
endif()

if(CONFIG_RPTUN)
  list(APPEND SRCS k230_rptun.c)
endif()

target_sources(arch PRIVATE ${SRCS})

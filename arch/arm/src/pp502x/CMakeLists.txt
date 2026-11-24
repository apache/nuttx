# ##############################################################################
# arch/arm/src/pp502x/CMakeLists.txt
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

# The start-up file has the vector table (its reset slot must be a relative
# branch), so it replaces the common head and vector table.

get_target_property(NUTTX_SRCS nuttx SOURCES)
list(FILTER NUTTX_SRCS EXCLUDE REGEX "arm_head\\.S$")
set_property(TARGET nuttx PROPERTY SOURCES ${NUTTX_SRCS})

get_target_property(ARCH_SRCS arch SOURCES)
list(FILTER ARCH_SRCS EXCLUDE REGEX "arm_vectortab\\.S$")
set_property(TARGET arch PROPERTY SOURCES ${ARCH_SRCS})

target_sources(nuttx PRIVATE pp502x_head.S)

set(SRCS pp502x_allocateheap.c pp502x_cache.c pp502x_decodeirq.c pp502x_irq.c
         pp502x_lowsetup.c pp502x_serial.c)

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS pp502x_timerisr.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS pp502x_idle.c)
endif()

target_sources(arch PRIVATE ${SRCS})

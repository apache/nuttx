# ##############################################################################
# arch/risc-v/src/k210/CMakeLists.txt
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

set(SRCS k210_head.S k210_start.c k210_irq.c k210_irq_dispatch.c)

list(APPEND SRCS k210_timerisr.c k210_allocateheap.c k210_clockconfig.c)

list(APPEND SRCS k210_lowputc.c k210_serial.c k210_fpioa.c k210_gpiohs.c)

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS k210_userspace.c)
endif()

target_sources(arch PRIVATE ${SRCS})

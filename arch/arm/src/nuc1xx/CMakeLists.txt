# ##############################################################################
# arch/arm/src/nuc1xx/CMakeLists.txt
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

list(
  APPEND
  SRCS
  nuc_clockconfig.c
  nuc_gpio.c
  nuc_irq.c
  nuc_lowputc.c
  nuc_serial.c
  nuc_start.c)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS nuc_idle.c)
endif()

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS nuc_timerisr.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS nuc_userspace.c)
endif()

if(CONFIG_DEBUG_FEATURES)
  list(APPEND SRCS nuc_dumpgpio.c)
endif()

target_sources(arch PRIVATE ${SRCS})

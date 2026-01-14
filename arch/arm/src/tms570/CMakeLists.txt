# ##############################################################################
# arch/arm/src/tms570/CMakeLists.txt
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

set(SRCS
    tms570_boot.c
    tms570_clockconfig.c
    tms570_esm.c
    tms570_gio.c
    tms570_irq.c
    tms570_lowputc.c
    tms570_serial.c)

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS tms570_timerisr.c)
endif()

if(CONFIG_TMS570_SELFTEST)
  list(APPEND SRCS tms570_selftest.c)
endif()

if(CONFIG_TMS570_GIO_IRQ)
  list(APPEND SRCS tms570_gioirq.c)
endif()

target_sources(arch PRIVATE ${SRCS})

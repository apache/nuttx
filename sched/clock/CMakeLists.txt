# ##############################################################################
# sched/clock/CMakeLists.txt
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
    clock.c
    clock_initialize.c
    clock_getres.c
    clock_settime.c
    clock_gettime.c
    clock_realtime2absticks.c
    clock_systime_ticks.c
    clock_systime_timespec.c
    clock_sched_ticks.c
    clock_perf.c)

# Unless a driver with a more accurate version of up_*delay is enabled, always
# include the base delay definition (busy-loop) as a minimum-viable product. We
# don't want the weak references to conflict.

if(NOT CONFIG_TIMER_ARCH AND NOT CONFIG_ALARM_ARCH)
  list(APPEND SRCS clock_delay.c)
endif()

if(CONFIG_CLOCK_TIMEKEEPING)
  list(APPEND SRCS clock_timekeeping.c)
endif()

if(CONFIG_CLOCK_ADJTIME)
  list(APPEND SRCS clock_adjtime.c)
endif()

target_sources(sched PRIVATE ${SRCS})

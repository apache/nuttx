# ##############################################################################
# libs/libc/sched/CMakeLists.txt
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
    sched_getprioritymax.c
    sched_getprioritymin.c
    clock_ticks2time.c
    clock_time2ticks.c
    clock_timespec_add.c
    clock_timespec_subtract.c
    clock_getcpuclockid.c
    clock_getres.c
    task_cancelpt.c
    task_setcancelstate.c
    task_setcanceltype.c
    task_testcancel.c)

if(CONFIG_SMP)
  list(APPEND SRCS sched_cpucount.c)
endif()

if(CONFIG_SCHED_BACKTRACE)
  list(APPEND SRCS sched_dumpstack.c sched_backtrace.c)
endif()

if(NOT CONFIG_BUILD_KERNEL)
  list(APPEND SRCS task_startup.c)
endif()

target_sources(c PRIVATE ${SRCS})

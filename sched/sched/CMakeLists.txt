# ##############################################################################
# sched/sched/CMakeLists.txt
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
    sched_getfiles.c
    sched_addreadytorun.c
    sched_removereadytorun.c
    sched_addprioritized.c
    sched_mergeprioritized.c
    sched_mergepending.c
    sched_addblocked.c
    sched_removeblocked.c
    sched_gettcb.c
    sched_verifytcb.c
    sched_releasetcb.c
    sched_setparam.c
    sched_setpriority.c
    sched_getparam.c
    sched_setscheduler.c
    sched_getscheduler.c
    sched_yield.c
    sched_rrgetinterval.c
    sched_foreach.c
    sched_lock.c
    sched_unlock.c
    sched_lockcount.c
    sched_idletask.c
    sched_self.c
    sched_get_stackinfo.c
    sched_get_tls.c
    sched_sysinfo.c
    sched_reprioritizertr.c
    sched_get_stateinfo.c)

if(CONFIG_PRIORITY_INHERITANCE)
  list(APPEND SRCS sched_reprioritize.c)
endif()

if(CONFIG_SMP)
  list(
    APPEND
    SRCS
    sched_cpuselect.c
    sched_cpupause.c
    sched_getcpu.c
    sched_getaffinity.c
    sched_setaffinity.c
    sched_process_delivered.c)
endif()

if(CONFIG_SIG_SIGSTOP_ACTION)
  list(APPEND SRCS sched_suspend.c)
endif()

if(CONFIG_SCHED_WAITPID)
  list(APPEND SRCS sched_waitpid.c)
  if(CONFIG_SCHED_HAVE_PARENT)
    list(APPEND SRCS sched_waitid.c sched_wait.c)
  endif()
endif()

if(NOT "${CONFIG_RR_INTERVAL}" STREQUAL "0")
  list(APPEND SRCS sched_roundrobin.c)
endif()

if(CONFIG_SCHED_SPORADIC)
  list(APPEND SRCS sched_sporadic.c)
endif()

if(CONFIG_SCHED_SUSPENDSCHEDULER)
  list(APPEND SRCS sched_suspendscheduler.c)
endif()

if(NOT "${CONFIG_RR_INTERVAL}" STREQUAL "0")
  list(APPEND SRCS sched_resumescheduler.c)
elseif(CONFIG_SCHED_RESUMESCHEDULER)
  list(APPEND SRCS sched_resumescheduler.c)
endif()

if(NOT CONFIG_SCHED_CPULOAD_NONE)
  list(APPEND SRCS sched_cpuload.c)
  if(CONFIG_CPULOAD_ONESHOT)
    list(APPEND SRCS sched_cpuload_oneshot.c)
  endif()
  if(CONFIG_CPULOAD_PERIOD)
    list(APPEND SRCS sched_cpuload_period.c)
  endif()
endif()

if(CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS sched_timerexpiration.c)
else()
  list(APPEND SRCS sched_processtimer.c)
endif()

if(CONFIG_SCHED_CRITMONITOR)
  list(APPEND SRCS sched_critmonitor.c)
endif()

if(CONFIG_SCHED_BACKTRACE)
  list(APPEND SRCS sched_backtrace.c)
endif()

if(CONFIG_SCHED_DUMP_ON_EXIT)
  list(APPEND SRCS sched_dumponexit.c)
endif()

if(CONFIG_SMP)
  list(APPEND SRCS sched_smp.c)
endif()

target_sources(sched PRIVATE ${SRCS})

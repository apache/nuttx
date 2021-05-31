############################################################################
# sched/sched/Make.defs
#
#   Copyright (C) 2014, 2018 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
set(SRCS)

list(APPEND SRCS
  sched_getfiles.c
  sched_addreadytorun.c sched_removereadytorun.c
  sched_addprioritized.c sched_mergeprioritized.c sched_mergepending.c
  sched_addblocked.c sched_removeblocked.c
  sched_gettcb.c sched_verifytcb.c sched_releasetcb.c
  sched_getstreams.c sched_backtrace.c
  sched_setparam.c sched_setpriority.c sched_getparam.c
  sched_setscheduler.c sched_getscheduler.c
  sched_yield.c sched_rrgetinterval.c sched_foreach.c
  sched_lock.c sched_unlock.c sched_lockcount.c
  sched_idletask.c sched_self.c sched_get_stackinfo.c)

if (CONFIG_PRIORITY_INHERITANCE)
  list(APPEND SRCS sched_reprioritize.c)
endif()

if (CONFIG_SMP)
  list(APPEND SRCS sched_cpuselect.c sched_cpupause.c sched_getcpu.c
    sched_getaffinity.c sched_setaffinity.c)
endif()

if (CONFIG_SIG_SIGSTOP_ACTION)
  list(APPEND SRCS sched_suspend.c sched_continue.c)
endif()

if (CONFIG_SCHED_WAITPID)
  list(APPEND SRCS sched_waitpid.c)
  if (CONFIG_SCHED_HAVE_PARENT)
    list(APPEND SRCS sched_waitid.c sched_wait.c)
  endif()
endif()

if (NOT "${CONFIG_RR_INTERVAL}" STREQUAL "0")
  list(APPEND SRCS sched_roundrobin.c)
endif()

if (CONFIG_SCHED_SPORADIC)
  list(APPEND SRCS sched_sporadic.c)
endif()

if (CONFIG_SCHED_SUSPENDSCHEDULER)
  list(APPEND SRCS sched_suspendscheduler.c)
endif()

if (NOT "${CONFIG_RR_INTERVAL}" STREQUAL "0")
  list(APPEND SRCS sched_resumescheduler.c)
elseif (CONFIG_SCHED_RESUMESCHEDULER)
  list(APPEND SRCS sched_resumescheduler.c)
endif()

if (CONFIG_SCHED_CPULOAD)
  list(APPEND SRCS sched_cpuload.c)
  if (CONFIG_CPULOAD_ONESHOT)
    list(APPEND SRCS sched_cpuload_oneshot.c)
  endif()
  if (CONFIG_CPULOAD_PERIOD)
    list(APPEND SRCS sched_cpuload_period.c)
  endif()
endif()

if (CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS sched_timerexpiration.c)
else()
  list(APPEND SRCS sched_processtimer.c)
endif()

if (CONFIG_SMP)
  list(APPEND SRCS sched_thistask.c)
endif()

if (CONFIG_SCHED_INSTRUMENTATION)
  if (CONFIG_SCHED_INSTRUMENTATION_EXTERNAL)
    list(APPEND SRCS sched_note.c)
  endif()
endif()

if (CONFIG_SCHED_CRITMONITOR)
  list(APPEND SRCS sched_critmonitor.c)
endif()

target_sources(sched PRIVATE ${SRCS})

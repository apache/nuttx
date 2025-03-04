# ##############################################################################
# arch/x86_64/src/intel64/CMakeLists.txt
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
    intel64_saveusercontext.S
    intel64_fullcontextrestore.S
    intel64_vectors.S
    intel64_head.S
    intel64_createstack.c
    intel64_initialstate.c
    intel64_irq.c
    intel64_map_region.c
    intel64_regdump.c
    intel64_releasestack.c
    intel64_rtc.c
    intel64_restore_auxstate.c
    intel64_stackframe.c
    intel64_schedulesigaction.c
    intel64_sigdeliver.c
    intel64_usestack.c
    intel64_systemreset.c
    intel64_freq.c
    intel64_cache.c
    intel64_start.c
    intel64_handlers.c
    intel64_idle.c
    intel64_lowsetup.c
    intel64_serial.c
    intel64_rng.c
    intel64_check_capability.c
    intel64_cpu.c)

if(CONFIG_X86_64_UNWINDER_FRAME_POINTER)
  list(APPEND SRCS intel64_backtrace_fp.c)
endif()

if(CONFIG_STACK_COLORATION)
  list(APPEND SRCS intel64_checkstack.c)
endif()

if(CONFIG_ARCH_PERF_EVENTS)
  list(APPEND SRCS intel64_perf.c)
endif()

if(CONFIG_MM_PGALLOC)
  list(APPEND SRCS intel64_pgalloc.c)
endif()

if(CONFIG_ARCH_HAVE_TESTSET)
  list(APPEND SRCS intel64_testset.S)
endif()

if(CONFIG_SMP)
  list(APPEND SRCS intel64_cpuidlestack.c intel64_smpcall.c intel64_cpustart.c)
endif()

if(CONFIG_MULTBOOT2_FB_TERM)
  list(APPEND SRCS intel64_fbterm.c)
endif()

if(CONFIG_MULTBOOT2_FB)
  list(APPEND SRCS intel64_fb.c)
endif()

if(CONFIG_ARCH_INTEL64_HAVE_TSC)
  if(CONFIG_SCHED_TICKLESS)
    list(APPEND SRCS intel64_tsc_tickless.c)
  else()
    list(APPEND SRCS intel64_tsc_timerisr.c)
  endif()
  list(APPEND SRCS intel64_tsc_ndelay.c)
endif()

if(CONFIG_INTEL64_HPET)
  list(APPEND SRCS intel64_hpet.c)
endif()

if(CONFIG_ARCH_INTEL64_HPET_ALARM)
  list(APPEND SRCS intel64_hpet_alarm.c)
endif()

if(CONFIG_INTEL64_ONESHOT)
  list(APPEND SRCS intel64_oneshot.c intel64_oneshot_lower.c)
endif()

if(CONFIG_ARCH_FPU)
  list(APPEND SRCS intel64_fpucmp.c)
endif()

target_sources(arch PRIVATE ${SRCS})

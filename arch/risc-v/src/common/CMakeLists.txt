# ##############################################################################
# arch/risc-v/src/common/CMakeLists.txt
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

set(SRCS)

list(APPEND SRCS riscv_exception_common.S riscv_vectors.S)
list(APPEND SRCS riscv_saveusercontext.S)
list(APPEND SRCS riscv_allocateheap.c riscv_cpuidlestack.c)
list(APPEND SRCS riscv_cpuinfo.c riscv_createstack.c riscv_doirq.c
     riscv_exception.c)
list(APPEND SRCS riscv_exit.c riscv_getintstack.c riscv_getnewintctx.c)
list(APPEND SRCS riscv_initialize.c riscv_initialstate.c riscv_modifyreg32.c)
list(APPEND SRCS riscv_mtimer.c riscv_nputs.c riscv_registerdump.c)
list(APPEND SRCS riscv_releasestack.c riscv_schedulesigaction.c
     riscv_sigdeliver.c)
list(APPEND SRCS riscv_stackframe.c riscv_tcbinfo.c riscv_swint.c)
list(APPEND SRCS riscv_switchcontext.c riscv_usestack.c)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS riscv_idle.c)
endif()

if(NOT CONFIG_ALARM_ARCH)
  if(NOT CONFIG_TIMER_ARCH)
    list(APPEND SRCS riscv_mdelay.c riscv_udelay.c)
  endif()
endif()

if(CONFIG_SMP)
  list(APPEND SRCS riscv_smpcall.c riscv_cpustart.c)
endif()

if(CONFIG_ARCH_HAVE_MULTICPU)
  if(CONFIG_ARCH_USE_S_MODE)
    list(APPEND SRCS riscv_cpuindex.c)
  endif()
endif()

if(CONFIG_ARCH_RV_CPUID_MAP)
  list(APPEND SRCS riscv_cpuidmap.c)
endif()

if(CONFIG_RISCV_MISALIGNED_HANDLER)
  list(APPEND SRCS riscv_misaligned.c)
endif()

if(CONFIG_ARCH_HAVE_DEBUG)
  list(APPEND SRCS riscv_debug.c)
endif()

if(NOT CONFIG_BUILD_FLAT)
  list(APPEND SRCS riscv_task_start.c riscv_pthread_start.c
       riscv_signal_dispatch.c)
  if(CONFIG_BUILD_PROTECTED)
    target_sources(arch_interface PRIVATE riscv_signal_handler.S)
  endif()
endif()

if(CONFIG_SCHED_BACKTRACE)
  list(APPEND SRCS riscv_backtrace.c)
endif()

if(CONFIG_STACK_COLORATION)
  list(APPEND SRCS riscv_checkstack.c)
endif()

if(CONFIG_ARCH_HAVE_FORK)
  list(APPEND SRCS fork.S riscv_fork.c)
endif()

if(CONFIG_SCHED_THREAD_LOCAL)
  list(APPEND SRCS riscv_tls.c)
endif()

if(CONFIG_ARCH_FPU)
  list(APPEND SRCS riscv_fpu.S riscv_fpucmp.c)
endif()

if(CONFIG_ARCH_RV_ISA_V)
  list(APPEND SRCS riscv_vpu.S)
endif()

if(CONFIG_ARCH_RV_HAVE_APLIC)
  list(APPEND SRCS riscv_aplic.c)
endif()

if(CONFIG_ARCH_RV_HAVE_IMSIC)
  list(APPEND SRCS riscv_imsic.c)
endif()

if(CONFIG_RISCV_SEMIHOSTING_HOSTFS)
  list(APPEND SRCS riscv_semihost.S riscv_hostfs.c)
endif()

if(CONFIG_ARCH_USE_MPU)
  list(APPEND SRCS riscv_pmp.c)
endif()

if(CONFIG_ARCH_USE_MMU)
  list(APPEND SRCS riscv_mmu.c)
endif()

if(CONFIG_ARCH_KERNEL_STACK)
  list(APPEND SRCS riscv_addrenv_kstack.c)
endif()

if(CONFIG_ARCH_ADDRENV)
  list(APPEND SRCS riscv_addrenv.c riscv_pgalloc.c riscv_addrenv_perms.c)
  list(APPEND SRCS riscv_addrenv_utils.c riscv_addrenv_shm.c)
endif()

if(CONFIG_RISCV_PERCPU_SCRATCH)
  list(APPEND SRCS riscv_percpu.c)
endif()

if(CONFIG_ARCH_USE_S_MODE)
  add_subdirectory(supervisor)
endif()

target_sources(arch PRIVATE ${SRCS})

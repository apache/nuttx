# ##############################################################################
# arch/arm64/src/common/CMakeLists.txt
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

set(SRCS arm64_head.S)

# TODO: support kernel startup obj ctr0

list(APPEND SRCS arm64_vector_table.S arm64_vectors.S arm64_smccc.S)
list(APPEND SRCS arm64_fork_func.S)

# Common C source files ( OS call up_xxx)
list(APPEND SRCS arm64_initialize.c arm64_initialstate.c arm64_boot.c)
list(APPEND SRCS arm64_nputs.c arm64_copystate.c arm64_createstack.c)
list(APPEND SRCS arm64_releasestack.c arm64_stackframe.c arm64_usestack.c)
list(APPEND SRCS arm64_exit.c arm64_fork.c)
list(APPEND SRCS arm64_schedulesigaction.c arm64_sigdeliver.c)
list(APPEND SRCS arm64_getintstack.c arm64_registerdump.c)
list(APPEND SRCS arm64_perf.c arm64_tcbinfo.c)

# Common C source files ( hardware BSP )
list(APPEND SRCS arm64_arch_timer.c arm64_cache.c)
list(APPEND SRCS arm64_doirq.c arm64_fatal.c)
list(APPEND SRCS arm64_syscall.c)

# Use common heap allocation for now (may need to be customized later)
list(APPEND SRCS arm64_allocateheap.c)

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS arm64_idle.c)
endif()

if(CONFIG_ARM64_GIC_VERSION EQUAL 3)
  list(APPEND SRCS arm64_gicv3.c)
endif()

if(CONFIG_ARM64_GIC_VERSION EQUAL 2)
  list(APPEND SRCS arm64_gicv2.c)
  if(CONFIG_ARM64_GICV2M)
    list(APPEND SRCS arm64_gicv2m.c)
  endif()
endif()

if(CONFIG_ARCH_HAVE_MMU)
  list(APPEND SRCS arm64_mmu.c)
endif()

if(CONFIG_ARCH_HAVE_MPU)
  list(APPEND SRCS arm64_mpu.c)
endif()

if(CONFIG_ARM64_PSCI)
  list(APPEND SRCS arm64_cpu_psci.c)
endif()

if(CONFIG_SMP)
  list(APPEND SRCS arm64_cpuidlestack.c arm64_cpustart.c)
  list(APPEND SRCS arm64_cpupause.c)
endif()

if(CONFIG_BUILD_KERNEL)
  list(APPEND SRCS arm64_task_start.c arm64_pthread_start.c
       arm64_signal_dispatch.c)
endif()

if(CONFIG_ARCH_KERNEL_STACK)
  list(APPEND SRCS arm64_addrenv_kstack.c)
endif()

if(CONFIG_ARCH_ADDRENV)
  list(APPEND SRCS arm64_addrenv.c arm64_pgalloc.c arm64_addrenv_perms.c)
  list(APPEND SRCS arm64_addrenv_utils.c arm64_addrenv_shm.c)
  list(APPEND SRCS arm64_addrenv_pgmap.c)
  if(CONFIG_ARCH_STACK_DYNAMIC)
    list(APPEND SRCS arm64_addrenv_ustack.c)
  endif()
endif()

if(CONFIG_MM_PGALLOC)
  list(APPEND SRCS arm64_physpgaddr.c)
  if(CONFIG_ARCH_PGPOOL_MAPPING)
    list(APPEND SRCS arm64_virtpgaddr.c)
  endif()
endif()

if(CONFIG_ARCH_FPU)
  list(APPEND SRCS arm64_fpu.c)
  list(APPEND SRCS arm64_fpu_func.S)
endif()

if(CONFIG_STACK_COLORATION)
  list(APPEND SRCS arm64_checkstack.c)
endif()

if(CONFIG_SCHED_BACKTRACE)
  list(APPEND SRCS arm64_backtrace.c)
endif()

if(CONFIG_ARM64_SEMIHOSTING_HOSTFS)
  list(APPEND SRCS arm64_hostfs.c)
endif()

target_sources(arch PRIVATE ${SRCS})

# ##############################################################################
# arch/arm/src/armv7-a/CMakeLists.txt
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

# The vector table is the "head" object, i.e., the one that must forced into the
# link in order to draw in all of the other components

set(SRCS arm_vectortab.S)

# Common assembly language files

list(APPEND SRCS arm_cpuhead.S arm_vectors.S arm_saveusercontext.S)

# Common C source files

list(
  APPEND
  SRCS
  arm_cache.c
  arm_cpuinfo.c
  arm_dataabort.c
  arm_doirq.c
  arm_gicv2.c
  arm_gicv2_dump.c
  arm_initialstate.c
  arm_mmu.c
  arm_prefetchabort.c
  arm_schedulesigaction.c
  arm_sigdeliver.c
  arm_syscall.c
  arm_tcbinfo.c
  arm_undefinedinsn.c
  cp15_cacheops.c)

if(CONFIG_ARCH_PERF_EVENTS)
  list(APPEND SRCS arm_perf.c)
endif()

if(CONFIG_ARMV7A_GICv2M)
  list(APPEND SRCS arm_gicv2m.c)
endif()

if(CONFIG_ARMV7A_HAVE_PTM)
  list(APPEND SRCS arm_timer.c)
endif()

if(CONFIG_ARMV7A_L2CC_PL310)
  list(APPEND SRCS arm_l2cc_pl310.c)
endif()

if(CONFIG_LEGACY_PAGING)
  list(
    APPEND
    SRCS
    arm_allocpage.c
    arm_checkmapping.c
    arm_pginitialize.c
    arm_va2pte.c
    arm_pghead.S)
else()
  list(APPEND SRCS arm_head.S)
endif()

if(CONFIG_ARCH_ADDRENV)
  list(
    APPEND
    SRCS
    arm_addrenv.c
    arm_addrenv_utils.c
    arm_addrenv_perms.c
    arm_pgalloc.c
    arm_addrenv_pgmap.c)
  if(CONFIG_ARCH_STACK_DYNAMIC)
    list(APPEND SRCS arm_addrenv_ustack.c)
  endif()
  if(CONFIG_ARCH_KERNEL_STACK)
    list(APPEND SRCS arm_addrenv_kstack.c)
  endif()
  if(CONFIG_ARCH_VMA_MAPPING)
    list(APPEND SRCS arm_addrenv_shm.c)
  endif()
endif()

if(CONFIG_MM_PGALLOC)
  list(APPEND SRCS arm_physpgaddr.c)
  if(CONFIG_ARCH_PGPOOL_MAPPING)
    list(APPEND SRCS arm_virtpgaddr.c)
  endif()
endif()

if(CONFIG_ARCH_FPU)
  list(APPEND SRCS arm_fpucmp.c arm_fpuconfig.S)
endif()

if(CONFIG_SMP)
  list(APPEND SRCS arm_cpustart.c arm_cpupause.c arm_cpuidlestack.c arm_scu.c)
endif()

if(CONFIG_ARCH_HAVE_PSCI)
  list(APPEND SRCS arm_cpu_psci.c arm_smccc.S)
endif()

if(CONFIG_ARCH_HIPRI_INTERRUPT)
  list(APPEND SRCS arm_dofiq.c)
endif()

target_sources(arch PRIVATE ${SRCS})

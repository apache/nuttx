# ##############################################################################
# arch/arm/src/armv7-r/CMakeLists.txt
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

# The vector table

set(SRCS arm_vectortab.S)

# Common assembly language files

list(APPEND SRCS arm_head.S arm_vectors.S arm_saveusercontext.S)

# Common C source files

list(
  APPEND
  SRCS
  arm_cache.c
  arm_cpuinfo.c
  arm_dataabort.c
  arm_doirq.c
  arm_initialstate.c
  arm_prefetchabort.c
  arm_schedulesigaction.c
  arm_sigdeliver.c
  arm_syscall.c
  arm_tcbinfo.c
  arm_undefinedinsn.c
  arm_perf.c
  cp15_cacheops.c)

if(NOT CONFIG_ARCH_CHIP STREQUAL tms570)
  list(APPEND SRCS arm_gicv2.c arm_gicv2_dump.c)
endif()

if(CONFIG_ARMV7R_HAVE_PTM)
  list(APPEND SRCS arm_timer.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS arm_mpu.c)
endif()

if(CONFIG_ARMV7R_L2CC_PL310)
  list(APPEND SRCS arm_l2cc_pl310.c)
endif()

if(CONFIG_ARCH_FPU)
  list(APPEND SRCS arm_fpucmp.c arm_fpuconfig.S)
endif()

if(CONFIG_SMP)
  list(
    APPEND
    SRCS
    arm_cpuhead.S
    arm_cpustart.c
    arm_smpcall.c
    arm_cpuidlestack.c
    arm_scu.c)
endif()

target_sources(arch PRIVATE ${SRCS})

# ##############################################################################
# arch/arm/src/common/CMakeLists.txt
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

if(CONFIG_ARCH_TOOLCHAIN_IAR)
  set(ARCH_TOOLCHAIN_PATH iar)
else()
  set(ARCH_TOOLCHAIN_PATH gnu)
endif()

set(SRCS
    arm_allocateheap.c
    arm_createstack.c
    arm_exit.c
    arm_getintstack.c
    arm_initialize.c
    arm_lowputs.c
    arm_modifyreg8.c
    arm_modifyreg16.c
    arm_modifyreg32.c
    arm_nputs.c
    arm_releasestack.c
    arm_registerdump.c
    arm_stackframe.c
    arm_usestack.c
    arm_fork.c
    ${ARCH_TOOLCHAIN_PATH}/fork.S)

if(NOT CONFIG_ALARM_ARCH AND NOT CONFIG_TIMER_ARCH)
  list(APPEND SRCS arm_mdelay.c arm_udelay.c)
endif()

if(CONFIG_STACK_COLORATION)
  list(APPEND SRCS arm_checkstack.c)
endif()

if(NOT CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS arm_idle.c)
endif()

if(CONFIG_BUILD_PROTECTED OR CONFIG_BUILD_KERNEL)
  list(APPEND SRCS arm_task_start.c arm_pthread_start.c arm_signal_dispatch.c)

  if(CONFIG_BUILD_PROTECTED)
    target_sources(arch_interface
                   PRIVATE ${ARCH_TOOLCHAIN_PATH}/arm_signal_handler.S)
  endif()
endif()

if(CONFIG_ARM_SEMIHOSTING_SYSLOG)
  list(APPEND SRCS arm_semi_syslog.c)
endif()

if(CONFIG_ARM_SEMIHOSTING_HOSTFS)
  list(APPEND SRCS arm_hostfs.c)
endif()

if(CONFIG_SCHED_THREAD_LOCAL)
  list(APPEND SRCS arm_tls.c)
endif()

if(CONFIG_UNWINDER_FRAME_POINTER)
  list(APPEND SRCS arm_backtrace_fp.c)
endif()

if(CONFIG_UNWINDER_STACK_POINTER)
  list(APPEND SRCS arm_backtrace_sp.c)
endif()

if(CONFIG_UNWINDER_ARM)
  list(APPEND SRCS arm_backtrace_unwind.c)
endif()

if(CONFIG_ARCH_HAVE_FETCHADD)
  list(APPEND SRCS ${ARCH_TOOLCHAIN_PATH}/arm_fetchadd.S)
endif()

target_sources(arch PRIVATE ${SRCS})

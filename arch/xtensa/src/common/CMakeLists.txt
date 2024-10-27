# ##############################################################################
# arch/xtensa/src/common/CMakeLists.txt
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

# The start-up, "head", file.  May be either a .S or a .c file.
add_library(xtensa_head OBJECT)
nuttx_add_library_internal(xtensa_head)
get_property(
  definitions
  TARGET nuttx
  PROPERTY NUTTX_KERNEL_DEFINITIONS)
target_compile_definitions(xtensa_head PRIVATE ${definitions})

get_property(
  options
  TARGET nuttx
  PROPERTY NUTTX_KERNEL_COMPILE_OPTIONS)

set(HEAD_SRCS xtensa_vectors.S xtensa_window_vector.S xtensa_windowspill.S
              xtensa_int_handlers.S xtensa_user_handler.S)

target_sources(xtensa_head PRIVATE ${HEAD_SRCS})
target_include_directories(xtensa_head BEFORE PUBLIC ${NUTTX_CHIP_ABS_DIR})
target_link_libraries(nuttx PRIVATE $<TARGET_OBJECTS:xtensa_head>)

# Common Xtensa files (arch/xtensa/src/common)

list(
  APPEND
  SRCS
  xtensa_context.S
  xtensa_cpuint.S
  xtensa_panic.S
  xtensa_assert.c
  xtensa_cache.c
  xtensa_cpenable.c
  xtensa_cpuinfo.c
  xtensa_createstack.c
  xtensa_exit.c
  xtensa_getintstack.c
  xtensa_initialize.c
  xtensa_initialstate.c
  xtensa_irqdispatch.c
  xtensa_lowputs.c
  xtensa_mdelay.c
  xtensa_modifyreg8.c
  xtensa_modifyreg16.c
  xtensa_modifyreg32.c
  xtensa_mpu.c
  xtensa_nputs.c
  xtensa_oneshot.c
  xtensa_perf.c
  xtensa_releasestack.c
  xtensa_registerdump.c
  xtensa_sigdeliver.c
  xtensa_swint.c
  xtensa_stackframe.c
  xtensa_saveusercontext.c
  xtensa_schedsigaction.c
  xtensa_udelay.c
  xtensa_usestack.c
  xtensa_tcbinfo.c)

# Configuration-dependent common Xtensa files

if(CONFIG_ARCH_USE_TEXT_HEAP)
  list(APPEND SRCS xtensa_loadstore.S)
endif()

if(CONFIG_ARCH_FPU)
  list(APPEND SRCS xtensa_fpucmp.c)
endif()

if(CONFIG_SCHED_BACKTRACE)
  list(APPEND SRCS xtensa_backtrace.c)
endif()

if(CONFIG_SMP)
  list(APPEND SRCS xtensa_smpcall.c)
endif()

if(CONFIG_STACK_COLORATION)
  list(APPEND SRCS xtensa_checkstack.c)
endif()

if(CONFIG_XTENSA_SEMIHOSTING_HOSTFS)
  list(APPEND SRCS xtensa_simcall.S xtensa_hostfs.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  target_sources(arch_interface PRIVATE xtensa_signal_handler.S)
  list(APPEND SRCS xtensa_dispatch_syscall.S)
  list(APPEND SRCS xtensa_signal_dispatch.c xtensa_task_start.c
       xtensa_pthread_start.c)
endif()

target_sources(arch PRIVATE ${SRCS})

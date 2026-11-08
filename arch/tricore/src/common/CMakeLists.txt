# ##############################################################################
# arch/tricore/src/common/CMakeLists.txt
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
    tricore_allocateheap.c
    tricore_cache.c
    tricore_checkstack.c
    tricore_createstack.c
    tricore_csa.c
    tricore_exit.c
    tricore_getintstack.c
    tricore_idle.c
    tricore_initialize.c
    tricore_initialstate.c
    tricore_irq.c
    tricore_main.c
    tricore_nputs.c
    tricore_perf.c
    tricore_registerdump.c
    tricore_releasestack.c
    tricore_saveusercontext.c
    tricore_stackframe.c
    tricore_svcall.c
    tricore_tcbinfo.c
    tricore_trapcall.c
    tricore_systimer.c
    tricore_usestack.c)

if(CONFIG_SCHED_BACKTRACE)
  list(APPEND SRCS tricore_backtrace.c)
endif()

if(CONFIG_ENABLE_ALL_SIGNALS)
  list(APPEND SRCS tricore_schedulesigaction.c tricore_sigdeliver.c)
endif()

if(CONFIG_ARCH_USE_MPU)
  list(APPEND SRCS tricore_mpu.c)
endif()

if(CONFIG_ARCH_HAVE_FPU)
  list(APPEND SRCS tricore_fpu.c)
endif()

if(CONFIG_ARCH_HAVE_DEBUG)
  list(APPEND SRCS tricore_hwdebug.c)
endif()

target_sources(arch PRIVATE ${SRCS})
target_sources(nuttx PRIVATE tricore_doirq.c)

# The trap vectors are placed by the linker script and not referenced by symbol,
# link them directly instead of from the archive.

if(CONFIG_TRICORE_TOOLCHAIN_TASKING)
  target_sources(nuttx PRIVATE tasking/tricore_vectors.${ASMEXT})
  target_sources(nuttx PRIVATE tasking/tricore_head.${ASMEXT})
else()
  target_sources(nuttx PRIVATE gnu/tricore_vectors.${ASMEXT})
  target_sources(nuttx PRIVATE gnu/tricore_head.${ASMEXT})
endif()
target_include_directories(
  nuttx SYSTEM PRIVATE ${NUTTX_DIR}/include ${NUTTX_BINARY_DIR}/include
                       ${NUTTX_BINARY_DIR}/include_arch)
# TASKING toolchain: build a fake libc_fpu.a to override the toolchain's default
# C library and provide stubs for the symbols the linker still references (see
# tricore_libc.c for details).

if(CONFIG_TRICORE_TOOLCHAIN_TASKING)
  # iLLD calls _c_init, the TASKING copy table init, so take cinit.o from the
  # toolchain C library.  The rest of that library clashes with NuttX.

  if(CONFIG_ARCH_TC1V8)
    set(TASKING_LIBDIR tc18/dpfpu)
  else()
    set(TASKING_LIBDIR tc162)
  endif()

  get_filename_component(TASKING_BIN_DIR ${CMAKE_C_COMPILER} DIRECTORY)
  execute_process(
    COMMAND ${TASKING_BIN_DIR}/artc -x
            ${TASKING_BIN_DIR}/../lib/${TASKING_LIBDIR}/libc_fpu.a cinit.o
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  nuttx_add_kernel_library(c_fpu)
  target_sources(c_fpu PRIVATE tricore_libc.c
                               ${CMAKE_CURRENT_BINARY_DIR}/cinit.o)
  target_link_options(nuttx PRIVATE
                      --library-directory=${CMAKE_CURRENT_BINARY_DIR})

  # Scope the delete-unreferenced-sections optimization to the kernel image
  # only: applied globally it also strips sections of the user library that are
  # still needed.

  target_link_options(nuttx PRIVATE -OtxYcL)
  if(CONFIG_BUILD_PROTECTED)
    target_link_options(nuttx_user PRIVATE --optimize=0)
  endif()
endif()

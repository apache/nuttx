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
    tricore_mdelay.c
    tricore_nputs.c
    tricore_registerdump.c
    tricore_releasestack.c
    tricore_saveusercontext.c
    tricore_schedulesigaction.c
    tricore_sigdeliver.c
    tricore_stackframe.c
    tricore_svcall.c
    tricore_switchcontext.c
    tricore_tcbinfo.c
    tricore_trapcall.c
    tricore_systimer.c
    tricore_usestack.c)

if(CONFIG_SPINLOCK)
  list(APPEND SRCS tricore_testset.c)
endif()

set(IFXFLAGS -DIFX_CFG_EXTEND_TRAP_HOOKS -DIFX_USE_SW_MANAGED_INT)

target_sources(arch PRIVATE ${SRCS})
target_sources(nuttx PRIVATE tricore_doirq.c)
target_compile_options(arch PRIVATE ${IFXFLAGS})
target_compile_options(nuttx PRIVATE ${IFXFLAGS})
target_include_directories(
  nuttx SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_BINARY_DIR}/include
                       ${CMAKE_BINARY_DIR}/include_arch)

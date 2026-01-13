# ##############################################################################
# arch/arm/src/arm_a_r/CMakeLists.txt
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

if(CONFIG_ENABLE_ALL_SIGNALS)
  set(CSRCS arm_schedulesigaction.c arm_sigdeliver.c)
endif()

if(CONFIG_ARCH_HAVE_DEBUG)
  list(APPEND CSRCS arm_hwdebug.c)
endif()

if(CONFIG_LIB_SYSCALL)
  list(APPEND CSRCS arm_dispatch_syscall.S)
endif()

if(CONFIG_PAGING)
  list(APPEND CSRCS arm_checkmapping.c)
endif()

list(
  APPEND
  CSRCS
  arm_userfault.c
  arm_tcbinfo.c
  arm_doirq.c
  arm_dataabort.c
  arm_syscall.c
  arm_cpuidlestack.c
  arm_initialstate.c
  arm_undefinedinsn.c
  arm_allocpage.c
  arm_prefetchabort.c)

target_sources(arch PRIVATE ${CSRCS})

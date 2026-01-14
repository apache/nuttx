# ##############################################################################
# arch/arm/src/arm/CMakeLists.txt
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

set(SRCS arm_head.S)

list(
  APPEND
  SRCS
  arm_dataabort.c
  arm_doirq.c
  arm_initialstate.c
  arm_prefetchabort.c
  arm_schedulesigaction.c
  arm_sigdeliver.c
  arm_syscall.c
  arm_tcbinfo.c
  arm_undefinedinsn.c
  arm_cache.S
  arm_vectors.S
  arm_vectortab.S
  arm_saveusercontext.S)

if(CONFIG_PAGING)
  list(APPEND SRCS arm_pginitialize.c arm_checkmapping.c arm_allocpage.c
       arm_va2pte.c)
endif()

target_sources(arch PRIVATE ${SRCS})

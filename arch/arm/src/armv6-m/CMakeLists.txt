# ##############################################################################
# arch/arm/src/armv6-m/CMakeLists.txt
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
    arm_exception.S
    arm_saveusercontext.S
    arm_doirq.c
    arm_hardfault.c
    arm_initialstate.c
    arm_schedulesigaction.c
    arm_sigdeliver.c
    arm_svcall.c
    arm_systemreset.c
    arm_tcbinfo.c
    arm_trigger_irq.c
    arm_vectors.c)

if((DEFINED CONFIG_DEBUG_FEATURES AND CONFIG_DEBUG_FEATURES)
   OR (DEFINED CONFIG_ARM_COREDUMP_REGION AND CONFIG_ARM_COREDUMP_REGION))
  list(APPEND SRCS arm_dumpnvic.c)
endif()

if(CONFIG_ARCH_RAMVECTORS)
  list(APPEND SRCS arm_ramvec_initialize.c arm_ramvec_attach.c)
endif()

target_sources(arch PRIVATE ${SRCS})

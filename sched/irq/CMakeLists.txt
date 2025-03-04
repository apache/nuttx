# ##############################################################################
# sched/irq/CMakeLists.txt
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

set(SRCS irq_initialize.c irq_attach.c irq_attach_thread.c irq_attach_wqueue.c
         irq_dispatch.c irq_unexpectedisr.c)

if(CONFIG_SPINLOCK)
  list(APPEND SRCS irq_spinlock.c)
endif()

if(CONFIG_IRQCOUNT)
  list(APPEND SRCS irq_csection.c)
endif()

if(CONFIG_SCHED_IRQMONITOR)
  list(APPEND SRCS irq_foreach.c)
  if(CONFIG_FS_PROCFS)
    list(APPEND SRCS irq_procfs.c)
  endif()
endif()

if(CONFIG_IRQCHAIN)
  list(APPEND SRCS irq_chain.c)
endif()

if(CONFIG_IRQ_WORK_SECTION)
  target_compile_definitions(
    sched PRIVATE -DIRQ_WORK_SECTION="${CONFIG_IRQ_WORK_SECTION}")
endif()

target_sources(sched PRIVATE ${SRCS})

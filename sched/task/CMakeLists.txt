# ##############################################################################
# sched/task/CMakeLists.txt
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
    task_argvstr.c
    task_create.c
    task_init.c
    task_setup.c
    task_activate.c
    task_start.c
    task_delete.c
    task_exit.c
    task_exithook.c
    task_getgroup.c
    task_getpid.c
    task_prctl.c
    task_recover.c
    task_restart.c
    task_spawnparms.c
    task_cancelpt.c
    task_terminate.c
    task_gettid.c
    task_join.c
    exit.c)

if(CONFIG_SCHED_HAVE_PARENT)
  list(APPEND SRCS task_getppid.c task_reparent.c)
endif()

if(CONFIG_ARCH_HAVE_FORK)
  list(APPEND SRCS task_fork.c)
endif()

if(NOT CONFIG_BUILD_KERNEL)
  list(APPEND SRCS task_spawn.c)
endif()

if(NOT CONFIG_BINFMT_DISABLE)
  if(CONFIG_LIBC_EXECFUNCS)
    list(APPEND SRCS task_execve.c task_posixspawn.c)
  endif()
endif()

if(CONFIG_SCHED_STARTHOOK)
  list(APPEND SRCS task_starthook.c)
endif()

target_sources(sched PRIVATE ${SRCS})

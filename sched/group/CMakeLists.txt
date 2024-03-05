# ##############################################################################
# sched/group/CMakeLists.txt
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
    group_create.c
    group_join.c
    group_leave.c
    group_setupidlefiles.c
    group_setuptaskfiles.c
    group_foreachchild.c
    group_killchildren.c
    group_signal.c
    group_argvstr.c)

if(CONFIG_SCHED_HAVE_PARENT)
  if(CONFIG_SCHED_CHILD_STATUS)
    list(APPEND SRCS group_childstatus.c)
  endif()
elseif(CONFIG_SCHED_WAITPID)
  list(APPEND SRCS group_waiter.c)
endif()

if(CONFIG_SCHED_USER_IDENTITY)
  list(
    APPEND
    SRCS
    group_setuid.c
    group_setgid.c
    group_getuid.c
    group_getgid.c
    group_seteuid.c
    group_setegid.c
    group_geteuid.c
    group_getegid.c)
endif()

if(CONFIG_SIG_SIGSTOP_ACTION)
  list(APPEND SRCS group_suspendchildren.c group_continue.c)
endif()

if(CONFIG_BINFMT_LOADABLE)
  list(APPEND SRCS group_exitinfo.c)
endif()

if(CONFIG_MM_KERNEL_HEAP)
  list(APPEND SRCS group_malloc.c group_realloc.c group_zalloc.c group_free.c)
endif()

target_sources(sched PRIVATE ${SRCS})

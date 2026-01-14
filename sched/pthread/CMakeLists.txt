# ##############################################################################
# sched/pthread/CMakeLists.txt
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

if(NOT CONFIG_DISABLE_PTHREAD)

  set(SRCS
      pthread_create.c
      pthread_exit.c
      pthread_join.c
      pthread_detach.c
      pthread_getschedparam.c
      pthread_setschedparam.c
      pthread_mutexinit.c
      pthread_mutexdestroy.c
      pthread_mutextimedlock.c
      pthread_mutextrylock.c
      pthread_mutexunlock.c
      pthread_condwait.c
      pthread_condsignal.c
      pthread_condbroadcast.c
      pthread_condclockwait.c
      pthread_sigmask.c
      pthread_cancel.c
      pthread_completejoin.c
      pthread_findjoininfo.c
      pthread_release.c
      pthread_setschedprio.c)

  if(NOT CONFIG_PTHREAD_MUTEX_UNSAFE)
    list(APPEND SRCS pthread_mutex.c pthread_mutexconsistent.c)
  endif()

  if(CONFIG_SMP)
    list(APPEND SRCS pthread_setaffinity.c pthread_getaffinity.c)
  endif()

  target_sources(sched PRIVATE ${SRCS})
endif()

# ##############################################################################
# libs/libc/pthread/CMakeLists.txt
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

set(SRCS)

if(NOT CONFIG_TLS_NELEM EQUAL 0)
  list(APPEND SRCS pthread_keycreate.c pthread_setspecific.c
       pthread_getspecific.c pthread_keydelete.c)
endif()

if(NOT CONFIG_DISABLE_PTHREAD)
  # Add the pthread C files to the build

  list(
    APPEND
    SRCS
    pthread_attr_init.c
    pthread_attr_destroy.c
    pthread_attr_setschedpolicy.c
    pthread_attr_getschedpolicy.c
    pthread_attr_setinheritsched.c
    pthread_attr_getinheritsched.c
    pthread_attr_setdetachstate.c
    pthread_attr_getdetachstate.c
    pthread_attr_setstackaddr.c
    pthread_attr_getstackaddr.c
    pthread_attr_setstacksize.c
    pthread_attr_getstacksize.c
    pthread_attr_setstack.c
    pthread_attr_getstack.c
    pthread_attr_setschedparam.c
    pthread_attr_getschedparam.c
    pthread_barrierattr_init.c
    pthread_barrierattr_destroy.c
    pthread_barrierattr_getpshared.c
    pthread_barrierattr_setpshared.c
    pthread_barrierinit.c
    pthread_barrierdestroy.c
    pthread_condattr_init.c
    pthread_condattr_destroy.c
    pthread_condattr_getpshared.c
    pthread_condattr_setpshared.c
    pthread_condattr_setclock.c
    pthread_condattr_getclock.c
    pthread_condinit.c
    pthread_conddestroy.c
    pthread_condtimedwait.c
    pthread_create.c
    pthread_exit.c
    pthread_kill.c
    pthread_setname_np.c
    pthread_getname_np.c
    pthread_get_stackaddr_np.c
    pthread_get_stacksize_np.c
    pthread_mutexattr_init.c
    pthread_mutexattr_destroy.c
    pthread_mutexattr_getpshared.c
    pthread_mutexattr_setpshared.c
    pthread_mutexattr_setprotocol.c
    pthread_mutexattr_getprotocol.c
    pthread_mutexattr_settype.c
    pthread_mutexattr_gettype.c
    pthread_mutexattr_setrobust.c
    pthread_mutexattr_getrobust.c
    pthread_mutex_lock.c
    pthread_once.c
    pthread_yield.c
    pthread_atfork.c
    pthread_rwlockattr_init.c
    pthread_rwlockattr_destroy.c
    pthread_rwlockattr_getpshared.c
    pthread_rwlockattr_setpshared.c
    pthread_rwlock.c
    pthread_rwlock_rdlock.c
    pthread_rwlock_wrlock.c
    pthread_setcancelstate.c
    pthread_setcanceltype.c
    pthread_testcancel.c
    pthread_getcpuclockid.c)

  if(CONFIG_SMP)
    list(APPEND SRCS pthread_attr_getaffinity.c pthread_attr_setaffinity.c)
  endif()

  if(CONFIG_PTHREAD_SPINLOCKS)
    list(APPEND SRCS pthread_spinlock.c)
  endif()

  if(NOT CONFIG_PTHREAD_CLEANUP_STACKSIZE EQUAL 0)
    list(APPEND SRCS pthread_cleanup.c)
  endif()
endif()

target_sources(c PRIVATE ${SRCS})

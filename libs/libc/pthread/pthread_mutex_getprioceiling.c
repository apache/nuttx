/****************************************************************************
 * libs/libc/pthread/pthread_mutex_getprioceiling.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mutex.h>

#include <pthread.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_getprioceiling
 *
 * Description:
 *   Return the mutex priority ceiling from the mutex.
 *
 * Input Parameters:
 *   mutex - The mutex to query
 *   prioceiling - Location to return the mutex priority ceiling
 *
 * Returned Value:
 *   0, if the mutex type was successfully return in 'prioceiling', or
 *   EINVAL, if any NULL pointers provided.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_mutex_getprioceiling(FAR const pthread_mutex_t *mutex,
                                 FAR int *prioceiling)
{
#ifdef CONFIG_PRIORITY_PROTECT
#  ifdef CONFIG_PTHREAD_MUTEX_TYPES
  return -nxrmutex_getprioceiling(&mutex->mutex, prioceiling);
#  else
  return -nxmutex_getprioceiling(&mutex->mutex, prioceiling);
#  endif
#else
  return EINVAL;
#endif
}

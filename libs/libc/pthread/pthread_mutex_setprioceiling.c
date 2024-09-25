/****************************************************************************
 * libs/libc/pthread/pthread_mutex_setprioceiling.c
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
 * Name: pthread_mutex_setprioceiling
 *
 * Description:
 *   Set the priority ceiling of a mutex.
 *
 * Input Parameters:
 *   mutex - The mutex in which to set the mutex priority ceiling.
 *   prioceiling - The mutex priority ceiling value to set.
 *   old_ceiling - Location to return the mutex ceiling priority set before.
 *
 * Returned Value:
 *   0, indicating the mutex priority ceiling was successfully set, or
 *   EINVAL, indicating 'mutex' or 'old_ceiling' is NULL, or 'prioceiling'
 *   is out of range.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_mutex_setprioceiling(FAR pthread_mutex_t *mutex,
                                 int prioceiling, FAR int *old_ceiling)
{
  int ret = EINVAL;

#ifdef CONFIG_PRIORITY_PROTECT
  ret = pthread_mutex_lock(mutex);
  if (ret != OK)
    {
      return ret;
    }

#  ifdef CONFIG_PTHREAD_MUTEX_TYPES
  ret = -nxrmutex_setprioceiling(&mutex->mutex, prioceiling, old_ceiling);
#  else
  ret = -nxmutex_setprioceiling(&mutex->mutex, prioceiling, old_ceiling);
#  endif
  pthread_mutex_unlock(mutex);
#endif
  return ret;
}

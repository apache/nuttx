/****************************************************************************
 * libs/libc/pthread/pthread_barrierdestroy.c
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

#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_barrier_destroy
 *
 * Description:
 *   The pthread_barrier_destroy() function destroys the barrier referenced
 *   by 'barrier' and releases any resources used by the barrier. The effect
 *   of subsequent use of the barrier is undefined until the barrier is
 *   reinitialized by another call to pthread_barrier_init(). The result
 *   are undefined if pthread_barrier_destroy() is called when any thread is
 *   blocked on the barrier, or if this function is called with an
 *   uninitialized barrier.
 *
 * Input Parameters:
 *   barrier - barrier to be destroyed.
 *
 * Returned Value:
 *   0 (OK) on success or on of the following error numbers:
 *
 *   EBUSY  The implementation has detected an attempt to destroy a barrier
 *          while it is in use.
 *   EINVAL The value specified by barrier is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_barrier_destroy(FAR pthread_barrier_t *barrier)
{
  int semcount;
  int ret = OK;

  if (!barrier)
    {
      ret = EINVAL;
    }
  else
    {
      ret = sem_getvalue(&barrier->sem, &semcount);
      if (ret != OK)
        {
          return ret;
        }

      if (semcount < 0)
        {
          return EBUSY;
        }

      sem_destroy(&barrier->sem);
      barrier->count = 0;
    }

  return ret;
}

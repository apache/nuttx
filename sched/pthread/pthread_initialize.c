/****************************************************************************
 * sched/pthread/pthread_initialize.c
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

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/semaphore.h>

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_sem_take, pthread_sem_trytake, and
 *       pthread_sem_give
 *
 * Description:
 *   Support managed access to the private data sets.
 *
 *   REVISIT: These functions really do nothing more than match the return
 *   value of the semaphore functions (0 or -1 with errno set) to the
 *   return value of more pthread functions (0 or errno).  A better solution
 *   would be to use an internal version of the semaphore functions that
 *   return the error value in the correct form.
 *
 * Input Parameters:
 *  sem  - The semaphore to lock or unlock
 *  intr - false: ignore EINTR errors when locking; true treat EINTR as
 *         other errors by returning the errno value
 *
 * Returned Value:
 *   0 on success or an errno value on failure.
 *
 ****************************************************************************/

int pthread_sem_take(FAR sem_t *sem, FAR const struct timespec *abs_timeout,
                     bool intr)
{
  int ret;

  if (intr)
    {
      if (abs_timeout == NULL)
        {
          ret = nxsem_wait(sem);
        }
      else
        {
          ret = nxsem_timedwait(sem, abs_timeout);
        }
    }
  else
    {
      if (abs_timeout == NULL)
        {
          ret = nxsem_wait_uninterruptible(sem);
        }
      else
        {
          ret = nxsem_timedwait_uninterruptible(sem, abs_timeout);
        }
    }

  return -ret;
}

#ifdef CONFIG_PTHREAD_MUTEX_UNSAFE
int pthread_sem_trytake(sem_t *sem)
{
  int ret = EINVAL;

  /* Verify input parameters */

  DEBUGASSERT(sem != NULL);
  if (sem != NULL)
    {
      /* Try to take the semaphore */

      int status = nxsem_trywait(sem);
      ret = status < 0 ? -status : OK;
    }

  return ret;
}
#endif

int pthread_sem_give(sem_t *sem)
{
  int ret;

  /* Verify input parameters */

  DEBUGASSERT(sem != NULL);
  if (sem != NULL)
    {
      /* Give the semaphore */

      ret = nxsem_post(sem);
      if (ret < 0)
        {
          return -ret;
        }

      return OK;
    }
  else
    {
      /* NULL semaphore pointer! */

      return EINVAL;
    }
}

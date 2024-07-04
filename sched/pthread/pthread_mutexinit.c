/****************************************************************************
 * sched/pthread/pthread_mutexinit.c
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
#include <pthread.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/semaphore.h>

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_init
 *
 * Description:
 *   Create a mutex
 *
 * Input Parameters:
 *   mutex - A reference to the mutex to be initialized
 *   attr - Mutex attribute object to be used
 *
 * Returned Value:
 *   0 if successful.  Otherwise, an error code.
 *
 ****************************************************************************/

int pthread_mutex_init(FAR pthread_mutex_t *mutex,
                       FAR const pthread_mutexattr_t *attr)
{
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
  uint8_t type = PTHREAD_MUTEX_DEFAULT;
#endif
#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
#ifdef CONFIG_PTHREAD_MUTEX_DEFAULT_UNSAFE
  uint8_t flags = 0;
#else
  uint8_t flags = _PTHREAD_MFLAGS_ROBUST;
#endif
#endif
  int ret = OK;
  int status;

  sinfo("mutex=%p attr=%p\n", mutex, attr);

  if (!mutex)
    {
      ret = EINVAL;
    }
  else
    {
      /* Were attributes specified?  If so, use them */

      if (attr)
        {
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
          type    = attr->type;
#endif
#ifdef CONFIG_PTHREAD_MUTEX_BOTH
          flags  = attr->robust == PTHREAD_MUTEX_ROBUST ?
                   _PTHREAD_MFLAGS_ROBUST : 0;
#endif
        }

      /* Initialize the mutex like a semaphore with initial count = 1 */

      status = mutex_init(&mutex->mutex);
      if (status < 0)
        {
          ret = -status;
        }

#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
      /* Initial internal fields of the mutex */

      mutex->flink = NULL;
      mutex->flags = flags;
#endif

#ifdef CONFIG_PTHREAD_MUTEX_TYPES
      /* Set up attributes unique to the mutex type */

      mutex->type  = type;
#endif
    }

  sinfo("Returning %d\n", ret);
  return ret;
}

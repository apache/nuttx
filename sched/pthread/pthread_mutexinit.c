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
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_mutex_init(FAR pthread_mutex_t *mutex,
                       FAR const pthread_mutexattr_t *attr)
{
  int pshared = 0;
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
  uint8_t type = PTHREAD_MUTEX_DEFAULT;
#endif
#ifdef CONFIG_PRIORITY_INHERITANCE
#  ifdef PTHREAD_MUTEX_DEFAULT_PRIO_INHERIT
  uint8_t proto = PTHREAD_PRIO_INHERIT;
#  else
  uint8_t proto = PTHREAD_PRIO_NONE;
#  endif
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
          pshared = attr->pshared;
#ifdef CONFIG_PRIORITY_INHERITANCE
          proto   = attr->proto;
#endif
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
          type    = attr->type;
#endif
#ifdef CONFIG_PTHREAD_MUTEX_BOTH
          flags  = attr->robust == PTHREAD_MUTEX_ROBUST ?
                   _PTHREAD_MFLAGS_ROBUST : 0;
#endif
        }

      /* Indicate that the semaphore is not held by any thread. */

      mutex->pid = INVALID_PROCESS_ID;

      /* Initialize the mutex like a semaphore with initial count = 1 */

      status = nxsem_init((FAR sem_t *)&mutex->sem, pshared, 1);
      if (status < 0)
        {
          ret = -ret;
        }

#ifdef CONFIG_PRIORITY_INHERITANCE
      /* Initialize the semaphore protocol */

      status = nxsem_set_protocol((FAR sem_t *)&mutex->sem, proto);
      if (status < 0)
        {
          ret = -status;
        }
#endif

#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
      /* Initial internal fields of the mutex */

      mutex->flink  = NULL;

      mutex->flags  = flags;
#endif

#ifdef CONFIG_PTHREAD_MUTEX_TYPES
      /* Set up attributes unique to the mutex type */

      mutex->type   = type;
      mutex->nlocks = 0;
#endif
    }

  sinfo("Returning %d\n", ret);
  return ret;
}

/****************************************************************************
 * libs/libc/pthread/pthread_mutex_init.c
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

#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/pthread.h>

#include "pthread.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static mutex_t g_lazyinit_lock = NXMUTEX_INITIALIZER;
static const pthread_mutex_t g_mutex_initializer = PTHREAD_MUTEX_INITIALIZER;

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
  int status;

  sinfo("mutex=%p attr=%p\n", mutex, attr);

  if (!mutex)
    {
      return EINVAL;
    }

  /* Initialize the semaphore of type mutex */

  status = mutex_init(&mutex->mutex);
  if (status < 0)
    {
      return -status;
    }

  /* Were attributes specified?  If so, use them */

#ifdef CONFIG_PTHREAD_MUTEX_TYPES
  mutex->type = attr ? attr->type : PTHREAD_MUTEX_DEFAULT;
#endif
#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
  mutex->flink = NULL;
#  ifdef CONFIG_PTHREAD_MUTEX_BOTH
  mutex->flags = attr && attr->robust == PTHREAD_MUTEX_ROBUST ?
                 _PTHREAD_MFLAGS_ROBUST : 0;
#  else
  mutex->flags = _PTHREAD_MFLAGS_ROBUST;
#  endif
#endif

#if defined(CONFIG_PRIORITY_INHERITANCE) || defined(CONFIG_PRIORITY_PROTECT)
  if (attr)
    {
      status = mutex_set_protocol(&mutex->mutex,
                                  SEM_TYPE_MUTEX | attr->proto);
      if (status < 0)
        {
          mutex_destroy(&mutex->mutex);
          return -status;
        }

#  ifdef CONFIG_PRIORITY_PROTECT
      if (attr->proto == PTHREAD_PRIO_PROTECT)
        {
          status = mutex_setprioceiling(&mutex->mutex, attr->ceiling, NULL);
          if (status < 0)
            {
              mutex_destroy(&mutex->mutex);
              return -status;
            }
        }
#  endif
    }
  else
    {
      /* Set the default priority inheritance and protect flags according to
       * config flags.
       */

      mutex_set_protocol(&mutex->mutex,
                         SEM_TYPE_MUTEX | PTHREAD_MUTEX_DEFAULT_PRIO_FLAGS);
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: pthread_mutex_lazyinit
 *
 * Description:
 *   Initialize a zero-filled mutex as if it had been set to
 *   PTHREAD_MUTEX_INITIALIZER.  The holder field is stored last, so other
 *   threads keep coming here until the rest of the mutex is set up.
 *
 * Input Parameters:
 *   mutex - A reference to the zero-filled mutex
 *
 ****************************************************************************/

void pthread_mutex_lazyinit(FAR pthread_mutex_t *mutex)
{
  pthread_mutex_t init = g_mutex_initializer;
  FAR sem_t *sem = PTHREAD_MUTEX_SEM(mutex);

  nxmutex_lock(&g_lazyinit_lock);

  /* Zero flags tells a zero-filled mutex apart from one held by TID 0 */

  if (sem->flags == 0)
    {
      PTHREAD_MUTEX_SEM(&init)->val.mholder = 0;
      memcpy(mutex, &init, sizeof(init));
      atomic_set_release(NXSEM_MHOLDER(sem), NXSEM_NO_MHOLDER);
    }

  nxmutex_unlock(&g_lazyinit_lock);
}

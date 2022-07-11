/****************************************************************************
 * libs/libc/pthread/pthread_barrierinit.c
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
 * Name: pthread_barrier_init
 *
 * Description:
 *   The pthread_barrier_init() function allocates any resources required to
 *   use the barrier referenced by 'barrier' and initialized the barrier
 *   with the attributes referenced by attr.  If attr is NULL, the default
 *   barrier attributes will be used. The results are undefined if
 *   pthread_barrier_init() is called when any thread is blocked on the
 *   barrier. The results are undefined if a barrier is used without first
 *   being initialized. The results are undefined if pthread_barrier_init()
 *   is called specifying an already initialized barrier.
 *
 * Input Parameters:
 *   barrier - the barrier to be initialized
 *   attr - barrier attributes to be used in the initialization.
 *   count - the count to be associated with the barrier.  The count
 *     argument specifies the number of threads that must call
 *     pthread_barrier_wait() before any of them successfully return from
 *     the call.  The value specified by count must be greater than zero.
 *
 * Returned Value:
 *   0 (OK) on success or on of the following error numbers:
 *
 *   EAGAIN The system lacks the necessary resources to initialize another
 *          barrier.  EINVAL The barrier reference is invalid, or the values
 *          specified by attr are invalid, or the value specified by count
 *          is equal to zero.
 *   ENOMEM Insufficient memory exists to initialize the barrier.
 *   EBUSY  The implementation has detected an attempt to reinitialize a
 *          barrier while it is in use.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_barrier_init(FAR pthread_barrier_t *barrier,
                         FAR const pthread_barrierattr_t *attr,
                         unsigned int count)
{
  int ret = OK;

  UNUSED(attr);

  if (!barrier || count == 0)
    {
      ret = EINVAL;
    }
  else
    {
      sem_init(&barrier->sem, 0, 0);
      sem_setprotocol(&barrier->sem, SEM_PRIO_NONE);
      barrier->count = count;
    }

  return ret;
}

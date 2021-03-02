/****************************************************************************
 * libs/libc/pthread/pthread_condinit.c
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
#include <debug.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_cond_init
 *
 * Description:
 *   A thread can create condition variables.
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

int pthread_cond_init(FAR pthread_cond_t *cond,
                      FAR const pthread_condattr_t *attr)
{
  int ret = OK;

  sinfo("cond=0x%p attr=0x%p\n", cond, attr);

  if (cond == NULL)
    {
      ret = EINVAL;
    }

  /* Initialize the semaphore contained in the condition structure with
   * initial count = 0
   */

  else if (sem_init(&cond->sem, 0, 0) != OK)
    {
      ret = get_errno();
    }
  else
    {
      /* The contained semaphore is used for signaling and, hence, should
       * not have priority inheritance enabled.
       */

      sem_setprotocol(&cond->sem, SEM_PRIO_NONE);

      cond->clockid = attr ? attr->clockid : CLOCK_REALTIME;
    }

  sinfo("Returning %d\n", ret);
  return ret;
}

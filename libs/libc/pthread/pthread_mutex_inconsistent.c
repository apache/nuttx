/****************************************************************************
 * libs/libc/pthread/pthread_mutex_inconsistent.c
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

#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/pthread.h>
#include <nuttx/atomic.h>

#include "pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_inconsistent
 *
 * Description:
 *   This function is called when a pthread is terminated via either
 *   pthread_exit() or pthread_cancel().  It will check for any mutexes
 *   held by exiting thread.  It will mark them as inconsistent and
 *   then wake up the highest priority waiter for the mutex.  That
 *   instance of pthread_mutex_lock() will then return EOWNERDEAD.
 *
 * Input Parameters:
 *   tcb -- a reference to the TCB of the exiting pthread.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void pthread_mutex_inconsistent(FAR struct tls_info_s *tls)
{
  FAR struct pthread_mutex_s *mutex;

  /* Remove and process each mutex held by this task */

  while (tls->tl_mhead != NULL)
    {
      /* Remove the mutex from the TCB list */

      mutex         = tls->tl_mhead;
      tls->tl_mhead = mutex->flink;
      mutex->flink  = NULL;

      /* Mark the mutex as INCONSISTENT and wake up any waiting thread */

      mutex->flags |= _PTHREAD_MFLAGS_INCONSISTENT;
      mutex_reset(&mutex->mutex);
    }
}

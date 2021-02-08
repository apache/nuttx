/****************************************************************************
 * sched/pthread/pthread_mutexinconsistent.c
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

#include "pthread/pthread.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_mutex_inconsistent
 *
 * Description:
 *   This function is called when a pthread is terminated via either
 *   pthread_exit() or pthread_cancel().  It will check for any mutexes
 *   held by exitting thread.  It will mark them as inconsistent and
 *   then wake up the highest priority waiter for the mutex.  That
 *   instance of pthread_mutex_lock() will then return EOWNERDEAD.
 *
 * Input Parameters:
 *   tcb -- a reference to the TCB of the exitting pthread.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void pthread_mutex_inconsistent(FAR struct tcb_s *tcb)
{
  FAR struct pthread_mutex_s *mutex;
  irqstate_t flags;

  DEBUGASSERT(tcb != NULL);

  sched_lock();

  /* Remove and process each mutex held by this task */

  while (tcb->mhead != NULL)
    {
      /* Remove the mutex from the TCB list */

      flags        = enter_critical_section();
      mutex        = tcb->mhead;
      tcb->mhead   = mutex->flink;
      mutex->flink = NULL;
      leave_critical_section(flags);

      /* Mark the mutex as INCONSISTENT and wake up any waiting thread */

      mutex->flags |= _PTHREAD_MFLAGS_INCONSISTENT;
      pthread_sem_give(&mutex->sem);
    }

  sched_unlock();
}

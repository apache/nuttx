/****************************************************************************
 * sched/pthread/pthread_mutexinconsistent.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

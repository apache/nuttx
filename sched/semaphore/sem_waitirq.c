/****************************************************************************
 * sched/semaphore/sem_waitirq.c
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

#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/addrenv.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_wait_irq
 *
 * Description:
 *   This function is called when either:
 *
 *   1. A signal is received by a task that is waiting on a semaphore.
 *      According to the POSIX spec, "...the calling thread shall not return
 *      from the call to [nxsem_wait] until it either locks the semaphore or
 *      the call is interrupted by a signal."
 *   2. From logic associated with sem_timedwait().  This function is called
 *      when the timeout elapses without receiving the semaphore.
 *
 *   Note: this function should used within critical_section
 *
 * Input Parameters:
 *   wtcb    - A pointer to the TCB of the task that is waiting on a
 *             semphaphore, but has received a signal or timeout instead.
 *   errcode - EINTR if the semaphore wait was awakened by a signal;
 *             ETIMEDOUT if awakened by a timeout
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void nxsem_wait_irq(FAR struct tcb_s *wtcb, int errcode)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR sem_t *sem = wtcb->waitobj;

#ifdef CONFIG_ARCH_ADDRENV
  if (wtcb->addrenv_own)
    {
      addrenv_select(wtcb->addrenv_own);
    }
#endif

  /* It is possible that an interrupt/context switch beat us to the punch
   * and already changed the task's state.
   */

  DEBUGASSERT(sem != NULL && sem->semcount < 0);

  /* Restore the correct priority of all threads that hold references
   * to this semaphore.
   */

  nxsem_canceled(wtcb, sem);

  /* And increment the count on the semaphore.  This releases the count
   * that was taken by sem_post().  This count decremented the semaphore
   * count to negative and caused the thread to be blocked in the first
   * place.
   */

  sem->semcount++;

  /* Remove task from waiting list */

  dq_rem((FAR dq_entry_t *)wtcb, SEM_WAITLIST(sem));

#ifdef CONFIG_ARCH_ADDRENV
  if (wtcb->addrenv_own)
    {
      addrenv_restore();
    }
#endif

  /* Indicate that the wait is over. */

  wtcb->waitobj = NULL;

  /* Mark the errno value for the thread. */

  wtcb->errcode = errcode;

  /* Add the task to ready-to-run task list and
   * perform the context switch if one is needed
   */

  if (nxsched_add_readytorun(wtcb))
    {
      up_switch_context(wtcb, rtcb);
    }
}

/****************************************************************************
 * sched/semaphore/sem_waitirq.c
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

#ifdef CONFIG_BUILD_KERNEL
/****************************************************************************
 * Name: nxsem_waiter_select
 *
 * Description:
 *   Select the address environment of the waiter if its semaphore is in
 *   user memory and the caller runs in another one.  Call
 *   nxsem_waiter_restore() before the waiter becomes ready to run.
 *
 * Input Parameters:
 *   wtcb   - The TCB of the thread that waits on the semaphore
 *   oldenv - The location to save the current address environment
 *
 * Returned Value:
 *   true if the address environment changed.
 *
 * Assumptions:
 *   Called in a critical section.  Can be called from an interrupt handler.
 *
 ****************************************************************************/

bool nxsem_waiter_select(FAR struct tcb_s *wtcb,
                         FAR struct addrenv_s **oldenv)
{
  if (wtcb->addrenv_own == NULL ||
      wtcb->addrenv_own == this_task()->addrenv_curr ||
      !up_addrenv_user_vaddr((uintptr_t)wtcb->waitobj))
    {
      return false;
    }

  addrenv_select(wtcb->addrenv_own, oldenv);
  return true;
}

/****************************************************************************
 * Name: nxsem_waiter_restore
 *
 * Description:
 *   Undo nxsem_waiter_select().  Not addrenv_restore(): priority
 *   inheritance in an interrupt handler can change this_task().
 *
 * Input Parameters:
 *   rtcb   - this_task() at the time of nxsem_waiter_select()
 *   oldenv - The address environment that nxsem_waiter_select() saved
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsem_waiter_restore(FAR struct tcb_s *rtcb,
                          FAR struct addrenv_s *oldenv)
{
  addrenv_give(rtcb->addrenv_curr);
  rtcb->addrenv_curr = oldenv;
  addrenv_switch(this_task());
}
#endif

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
 *   Note: this function should be used within critical_section.
 *
 * Input Parameters:
 *   wtcb    - A pointer to the TCB of the task that is waiting on a
 *             semphaphore, but has received a signal or timeout instead.
 *   errcode - EINTR if the semaphore wait was awakened by a signal;
 *             ETIMEDOUT if awakened by a timeout.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *
 ****************************************************************************/

void nxsem_wait_irq(FAR struct tcb_s *wtcb, int errcode)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR sem_t *sem = wtcb->waitobj;
  FAR struct addrenv_s *oldenv = NULL;
  bool selected;
  bool wakeup;
  bool mutex;

  /* It is possible that an interrupt/context switch beat us to the punch
   * and already changed the task's state.
   */

  DEBUGASSERT(sem != NULL);

  /* The semaphore can be in the address environment of the waiter only */

  selected = nxsem_waiter_select(wtcb, &oldenv);

  mutex = NXSEM_IS_MUTEX(sem);

  DEBUGASSERT(mutex || atomic_read(NXSEM_COUNT(sem)) < 0);
  DEBUGASSERT(!mutex || NXSEM_MBLOCKING(atomic_read(NXSEM_MHOLDER(sem))));

  /* Mutex is never interrupted by a signal or canceled */

  wakeup = !(mutex && (errcode == EINTR || errcode == ECANCELED));
  if (wakeup)
    {
      /* Restore the correct priority of all threads that hold references
       * to this semaphore.
       */

      nxsem_canceled(wtcb, sem);

      /* Remove task from waiting list */

      dq_rem((FAR dq_entry_t *)wtcb, SEM_WAITLIST(sem));

      /* Restore the semaphore state changed by the previous sem_wait().
       * For mutexes, the holder TID remains unchanged, but the blocking bit
       * must be cleared when the timed-out task was the last waiter.
       */

      if (mutex)
        {
          if (dq_empty(SEM_WAITLIST(sem)))
            {
              atomic_and(NXSEM_MHOLDER(sem), ~NXSEM_MBLOCKING_BIT);
            }
        }
      else
        {
          atomic_add(NXSEM_COUNT(sem), 1);
        }
    }

  if (selected)
    {
      nxsem_waiter_restore(rtcb, oldenv);
    }

  if (wakeup)
    {
      /* Indicate that the wait is over. */

      wtcb->waitobj = NULL;

      /* Mark the errno value for the thread. */

      wtcb->errcode = errcode;

      /* Add the task to ready-to-run task list and
       * perform the context switch if one is needed
       */

      if (nxsched_add_readytorun(wtcb))
        {
          up_switch_context(this_task(), rtcb);
        }
    }
}

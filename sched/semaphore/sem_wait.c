/****************************************************************************
 * sched/semaphore/sem_wait.c
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

#include <stdbool.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/init.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mm/kmap.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_wait_slow
 *
 * Description:
 *   This function attempts to lock the semaphore referenced by 'sem' in
 *   slow mode.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   sem_wait except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 * Input Parameters:
 *   sem - Semaphore descriptor.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *   Possible returned errors:
 *
 *   - EINVAL:  Invalid attempt to get the semaphore
 *   - EINTR:   The wait was interrupted by the receipt of a signal.
 *
 ****************************************************************************/

int nxsem_wait_slow(FAR sem_t *sem)
{
  FAR struct tcb_s *rtcb = this_task();
  irqstate_t flags;
  int ret;
  bool unlocked;
  FAR struct tcb_s *htcb = NULL;
  bool mutex = NXSEM_IS_MUTEX(sem);

  /* The following operations must be performed with interrupts
   * disabled because nxsem_post() may be called from an interrupt
   * handler.
   */

  flags = enter_critical_section();

  /* Make sure we were supplied with a valid semaphore. */

  /* Check if the lock is available */

  if (mutex)
    {
      uint32_t mholder;

      /* We lock the mutex for us by setting the blocks bit,
       * this is all that is needed if we block
       */

      mholder = atomic_fetch_or(NXSEM_MHOLDER(sem), NXSEM_MBLOCKING_BIT);
      if (NXSEM_MACQUIRED(mholder))
        {
          /* htcb gets NULL if
           * - the only holder did exit (without posting first)
           * - the mutex was reset before
           * In both cases we simply acquire the mutex, thus recovering
           * from these situations.
           */

          htcb = nxsched_get_tcb(mholder & (~NXSEM_MBLOCKING_BIT));
        }

      unlocked = htcb == NULL;
    }
  else
    {
      unlocked = atomic_fetch_sub(NXSEM_COUNT(sem), 1) > 0;
    }

  if (unlocked)
    {
      /* It is, let the task take the semaphore. */

      ret = nxsem_protect_wait(sem);
      if (ret < 0)
        {
          if (mutex)
            {
              atomic_set(NXSEM_MHOLDER(sem), NXSEM_NO_MHOLDER);
            }
          else
            {
              atomic_fetch_add(NXSEM_COUNT(sem), 1);
            }

          leave_critical_section(flags);
          return ret;
        }

      /* For mutexes, we only add the holder to the tasks list at the
       * time when a task blocks on the mutex, for priority restoration
       */

      if (!mutex)
        {
          nxsem_add_holder(sem);
        }
    }

  /* The semaphore is NOT available, We will have to block the
   * current thread of execution.
   */

  else
    {
#ifdef CONFIG_PRIORITY_INHERITANCE
      uint8_t prioinherit = sem->flags & SEM_PRIO_MASK;
#endif

      /* First, verify that the task is not already waiting on a
       * semaphore
       */

      DEBUGASSERT(rtcb->waitobj == NULL);

#ifdef CONFIG_MM_KMAP
      sem = kmm_map_user(rtcb, sem, sizeof(*sem));
#endif

      /* Save the waited on semaphore in the TCB */

      rtcb->waitobj = sem;

      /* In case of a mutex, store the previous holder in the task's list */

      if (mutex)
        {
          nxsem_add_holder_tcb(htcb, sem);
        }

      /* If priority inheritance is enabled, then check the priority of
       * the holder of the semaphore.
       */

#ifdef CONFIG_PRIORITY_INHERITANCE
      if (prioinherit == SEM_PRIO_INHERIT)
        {
          /* Disable context switching.  The following operations must be
           * atomic with regard to the scheduler.
           */

          sched_lock();

          /* Boost the priority of any threads holding a count on the
           * semaphore.
           */

          nxsem_boost_priority(sem);
        }
#endif

      /* Set the errno value to zero (preserving the original errno)
       * value).  We reuse the per-thread errno to pass information
       * between sem_waitirq() and this functions.
       */

      rtcb->errcode = OK;

      /* Add the TCB to the prioritized semaphore wait queue, after
       * checking this is not the idle task - descheduling that
       * isn't going to end well.
       */

      DEBUGASSERT(!is_idle_task(rtcb));

      /* Remove the tcb task from the running list. */

      nxsched_remove_self(rtcb);

      /* Add the task to the specified blocked task list */

      rtcb->task_state = TSTATE_WAIT_SEM;
      nxsched_add_prioritized(rtcb, SEM_WAITLIST(sem));

      /* Now, perform the context switch */

      up_switch_context(this_task(), rtcb);

      /* When we resume at this point, either (1) the semaphore has been
       * assigned to this thread of execution, or (2) the semaphore wait
       * has been interrupted by a signal or a timeout.  We can detect
       * these latter cases be examining the per-thread errno value.
       *
       * In the event that the semaphore wait was interrupted by a
       * signal or a timeout, certain semaphore clean-up operations have
       * already been performed (see sem_waitirq.c).  Specifically:
       *
       * - nxsem_canceled() was called to restore the priority of all
       *   threads that hold a reference to the semaphore,
       * - The semaphore count was decremented, and
       * - tcb->waitobj was nullifed.
       *
       * It is necessary to do these things in sem_waitirq.c because a
       * long time may elapse between the time that the signal was issued
       * and this thread is awakened and this leaves a door open to
       * several race conditions.
       */

      /* Check if an error occurred while we were sleeping.  Expected
       * errors include EINTR meaning that we were awakened by a signal
       * or ETIMEDOUT meaning that the timer expired for the case of
       * sem_timedwait().
       *
       * If we were not awakened by a signal or a timeout, then
       * nxsem_add_holder() was called by logic in sem_wait() fore this
       * thread was restarted.
       */

      ret = rtcb->errcode != OK ? -rtcb->errcode : OK;

#ifdef CONFIG_MM_KMAP
      kmm_unmap(sem);
#endif

#ifdef CONFIG_PRIORITY_INHERITANCE
      if (prioinherit != 0)
        {
          sched_unlock();
        }
#endif
    }

  /* If this now holds the mutex, set the holder TID and the lock bit */

  if (mutex && ret == OK)
    {
      uint32_t blocking_bit =
        dq_empty(SEM_WAITLIST(sem)) ? 0 : NXSEM_MBLOCKING_BIT;

      atomic_set(NXSEM_MHOLDER(sem), ((uint32_t)rtcb->pid) | blocking_bit);
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: nxsem_wait_uninterruptible
 *
 * Description:
 *   This function is wrapped version of nxsem_wait(), which is
 *   uninterruptible and convenient for use.
 *
 * Parameters:
 *   sem - Semaphore descriptor.
 *
 * Return Value:
 *   Zero(OK)  - On success
 *   EINVAL    - Invalid attempt to get the semaphore
 *   ECANCELED - May be returned if the thread is canceled while waiting.
 *
 ****************************************************************************/

int nxsem_wait_uninterruptible(FAR sem_t *sem)
{
  int ret;

  do
    {
      /* Take the semaphore (perhaps waiting) */

      ret = nxsem_wait(sem);
    }
  while (ret == -EINTR);

  return ret;
}

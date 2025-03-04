/****************************************************************************
 * sched/semaphore/sem_post.c
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

#include <limits.h>
#include <errno.h>
#include <sched.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_post_slow
 *
 * Description:
 *   When a kernel thread has finished with a semaphore, it will call
 *   nxsem_post().  This function unlocks the semaphore referenced by sem
 *   by performing the semaphore unlock operation on that semaphore.
 *
 *   If the semaphore value resulting from this operation is positive, then
 *   no tasks were blocked waiting for the semaphore to become unlocked; the
 *   semaphore is simply incremented.
 *
 *   If the value of the semaphore resulting from this operation is zero,
 *   then one of the tasks blocked waiting for the semaphore shall be
 *   allowed to return successfully from its call to nxsem_wait().
 *
 * Input Parameters:
 *   sem - Semaphore descriptor
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

static int nxsem_post_slow(FAR sem_t *sem)
{
  FAR struct tcb_s *stcb = NULL;
  irqstate_t flags;
  int32_t sem_count;
#if defined(CONFIG_PRIORITY_INHERITANCE) || defined(CONFIG_PRIORITY_PROTECT)
  uint8_t proto;
#endif

  /* The following operations must be performed with interrupts
   * disabled because sem_post() may be called from an interrupt
   * handler.
   */

  flags = enter_critical_section();

  /* Check the maximum allowable value */

  sem_count = atomic_read(NXSEM_COUNT(sem));
  do
    {
      if (sem_count >= SEM_VALUE_MAX)
        {
          leave_critical_section(flags);
          return -EOVERFLOW;
        }
    }
  while (!atomic_try_cmpxchg_release(NXSEM_COUNT(sem), &sem_count,
                                     sem_count + 1));

  /* Perform the semaphore unlock operation, releasing this task as a
   * holder then also incrementing the count on the semaphore.
   *
   * NOTE:  When semaphores are used for signaling purposes, the holder
   * of the semaphore may not be this thread!  In this case,
   * nxsem_release_holder() will do nothing.
   *
   * In the case of a mutex this could be simply resolved since there is
   * only one holder but for the case of counting semaphores, there may
   * be many holders and if the holder is not this thread, then it is
   * not possible to know which thread/holder should be released.
   *
   * For this reason, it is recommended that priority inheritance be
   * disabled via nxsem_set_protocol(SEM_PRIO_NONE) when the semaphore is
   * initialized if the semaphore is to used for signaling purposes.
   */

  nxsem_release_holder(sem);

#if defined(CONFIG_PRIORITY_INHERITANCE) || defined(CONFIG_PRIORITY_PROTECT)
  /* Don't let any unblocked tasks run until we complete any priority
   * restoration steps.  Interrupts are disabled, but we do not want
   * the head of the ready-to-run list to be modified yet.
   *
   * NOTE: If this sched_lock is called from an interrupt handler, it
   * will do nothing.
   */

  proto = sem->flags & SEM_PRIO_MASK;
  if (proto != SEM_PRIO_NONE)
    {
      sched_lock();
    }
#endif

  /* If the result of semaphore unlock is non-positive, then
   * there must be some task waiting for the semaphore.
   */

  if (sem_count < 0)
    {
      /* Check if there are any tasks in the waiting for semaphore
       * task list that are waiting for this semaphore.  This is a
       * prioritized list so the first one we encounter is the one
       * that we want.
       */

      stcb = (FAR struct tcb_s *)dq_remfirst(SEM_WAITLIST(sem));

      if (stcb != NULL)
        {
          FAR struct tcb_s *rtcb = this_task();

          /* The task will be the new holder of the semaphore when
           * it is awakened.
           */

          nxsem_add_holder_tcb(stcb, sem);

          /* Stop the watchdog timer */

          if (WDOG_ISACTIVE(&stcb->waitdog))
            {
              wd_cancel(&stcb->waitdog);
            }

          /* Indicate that the wait is over. */

          stcb->waitobj = NULL;

          /* Add the task to ready-to-run task list and
           * perform the context switch if one is needed
           */

          if (nxsched_add_readytorun(stcb))
            {
              up_switch_context(stcb, rtcb);
            }
        }
#if 0 /* REVISIT:  This can fire on IOB throttle semaphore */
      else
        {
          /* This should not happen. */

          DEBUGPANIC();
        }
#endif
    }

  /* Check if we need to drop the priority of any threads holding
   * this semaphore.  The priority could have been boosted while they
   * held the semaphore.
   */

#if defined(CONFIG_PRIORITY_INHERITANCE) || defined(CONFIG_PRIORITY_PROTECT)
  if (proto != SEM_PRIO_NONE)
    {
      if (proto == SEM_PRIO_INHERIT)
        {
#ifdef CONFIG_PRIORITY_INHERITANCE
          nxsem_restore_baseprio(stcb, sem);
#endif
        }
      else if (proto == SEM_PRIO_PROTECT)
        {
          nxsem_protect_post(sem);
        }

      sched_unlock();
    }
#endif

  /* Interrupts may now be enabled. */

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_post
 *
 * Description:
 *   When a kernel thread has finished with a semaphore, it will call
 *   nxsem_post().  This function unlocks the semaphore referenced by sem
 *   by performing the semaphore unlock operation on that semaphore.
 *
 *   If the semaphore value resulting from this operation is positive, then
 *   no tasks were blocked waiting for the semaphore to become unlocked; the
 *   semaphore is simply incremented.
 *
 *   If the value of the semaphore resulting from this operation is zero,
 *   then one of the tasks blocked waiting for the semaphore shall be
 *   allowed to return successfully from its call to nxsem_wait().
 *
 * Input Parameters:
 *   sem - Semaphore descriptor
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

int nxsem_post(FAR sem_t *sem)
{
  DEBUGASSERT(sem != NULL);

  /* If this is a mutex, we can try to unlock the mutex in fast mode,
   * else try to get it in slow mode.
   */

#if !defined(CONFIG_PRIORITY_INHERITANCE) && !defined(CONFIG_PRIORITY_PROTECT)
  if (sem->flags & SEM_TYPE_MUTEX)
    {
      int32_t old = 0;
      if (atomic_try_cmpxchg_release(NXSEM_COUNT(sem), &old, 1))
        {
          return OK;
        }
    }
#endif

  return nxsem_post_slow(sem);
}

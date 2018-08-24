/****************************************************************************
 * sched/semaphore/sem_post.c
 *
 *   Copyright (C) 2007-2009, 2012-2013, 2016-2017 Gregory Nutt. All rights
 *     reserved.
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

#include <limits.h>
#include <semaphore.h>
#include <errno.h>
#include <sched.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"

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
  FAR struct tcb_s *stcb = NULL;
  irqstate_t flags;
  int ret = -EINVAL;

  /* Make sure we were supplied with a valid semaphore. */

  if (sem != NULL)
    {
      /* The following operations must be performed with interrupts
       * disabled because sem_post() may be called from an interrupt
       * handler.
       */

      flags = enter_critical_section();

      /* Perform the semaphore unlock operation, releasing this task as a
       * holder then also incrementing the count on the semaphore.
       *
       * NOTE:  When semaphores are used for signaling purposes, the holder
       * of the semaphore may not be this thread!  In this case,
       * nxsem_releaseholder() will do nothing.
       *
       * In the case of a mutex this could be simply resolved since there is
       * only one holder but for the case of counting semaphores, there may
       * be many holders and if the holder is not this thread, then it is
       * not possible to know which thread/holder should be released.
       *
       * For this reason, it is recommended that priority inheritance be
       * disabled via nxsem_setprotocol(SEM_PRIO_NONE) when the semahore is
       * initialixed if the semaphore is to used for signaling purposes.
       */

      DEBUGASSERT(sem->semcount < SEM_VALUE_MAX);
      nxsem_releaseholder(sem);
      sem->semcount++;

#ifdef CONFIG_PRIORITY_INHERITANCE
      /* Don't let any unblocked tasks run until we complete any priority
       * restoration steps.  Interrupts are disabled, but we do not want
       * the head of the read-to-run list to be modified yet.
       *
       * NOTE: If this sched_lock is called from an interrupt handler, it
       * will do nothing.
       */

      sched_lock();
#endif
      /* If the result of of semaphore unlock is non-positive, then
       * there must be some task waiting for the semaphore.
       */

      if (sem->semcount <= 0)
        {
          /* Check if there are any tasks in the waiting for semaphore
           * task list that are waiting for this semaphore. This is a
           * prioritized list so the first one we encounter is the one
           * that we want.
           */

          for (stcb = (FAR struct tcb_s *)g_waitingforsemaphore.head;
               (stcb && stcb->waitsem != sem);
               stcb = stcb->flink);

          if (stcb != NULL)
            {
              /* The task will be the new holder of the semaphore when
               * it is awakened.
               */

              nxsem_addholder_tcb(stcb, sem);

              /* It is, let the task take the semaphore */

              stcb->waitsem = NULL;

              /* Restart the waiting task. */

              up_unblock_task(stcb);
            }
          else
            {
              /* This should not happen. */

              DEBUGPANIC();
            }
        }

      /* Check if we need to drop the priority of any threads holding
       * this semaphore.  The priority could have been boosted while they
       * held the semaphore.
       */

#ifdef CONFIG_PRIORITY_INHERITANCE
      nxsem_restorebaseprio(stcb, sem);
      sched_unlock();
#endif
      ret = OK;

      /* Interrupts may now be enabled. */

      leave_critical_section(flags);
    }

  return ret;
}

/****************************************************************************
 * Name: sem_post
 *
 * Description:
 *   When a task has finished with a semaphore, it will call sem_post().
 *   This function unlocks the semaphore referenced by sem by performing the
 *   semaphore unlock operation on that semaphore.
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
 *   This function is a standard, POSIX application interface.  It will
 *   return zero (OK) if successful.  Otherwise, -1 (ERROR) is returned and
 *   the errno value is set appropriately.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

int sem_post(FAR sem_t *sem)
{
  int ret;

  ret = nxsem_post(sem);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

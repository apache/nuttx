/****************************************************************************
 * sched/semaphore/sem_reset.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
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

#include <semaphore.h>
#include <sched.h>
#include <assert.h>

#include <nuttx/irq.h>

#include "semaphore/semaphore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_reset
 *
 * Description:
 *   Reset a semaphore count to a specific value.  This is similar to part
 *   of the operation of nxsem_init().  But nxsem_reset() may need to wake up
 *   tasks waiting on a count.  This kind of operation is sometimes required
 *   within the OS (only) for certain error handling conditions.
 *
 * Input Parameters:
 *   sem   - Semaphore descriptor to be reset
 *   count - The requested semaphore count
 *
 * Returned Value:
 *   This is an internal OS interface, not available to applications, and
 *   hence follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_reset(FAR sem_t *sem, int16_t count)
{
  irqstate_t flags;

  DEBUGASSERT(sem != NULL && count >= 0);

  /* Don't allow any context switches that may result from the following
   * nxsem_post() operations.
   */

  sched_lock();

  /* Prevent any access to the semaphore by interrupt handlers while we are
   * performing this operation.
   */

  flags = enter_critical_section();

  /* A negative count indicates that the negated number of threads are
   * waiting to take a count from the semaphore.  Loop here, handing
   * out counts to any waiting threads.
   */

  while (sem->semcount < 0 && count > 0)
    {
      /* Give out one counting, waking up one of the waiting threads
       * and, perhaps, kicking off a lot of priority inheritance
       * logic (REVISIT).
       */

      DEBUGVERIFY(nxsem_post(sem));
      count--;
    }

  /* We exit the above loop with either (1) no threads waiting for the
   * (i.e., with sem->semcount >= 0).  In this case, 'count' holds the
   * the new value of the semaphore count.  OR (2) with threads still
   * waiting but all of the semaphore counts exhausted:  The current
   * value of sem->semcount is already correct in this case.
   */

  if (sem->semcount >= 0)
    {
      sem->semcount = count;
    }

  /* Allow any pending context switches to occur now */

  leave_critical_section(flags);
  sched_unlock();
  return OK;
}

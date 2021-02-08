/****************************************************************************
 * sched/semaphore/sem_reset.c
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

/****************************************************************************
 * sched/sched/sched_gettcb.c
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

#include <sys/types.h>

#include <sched.h>
#include <stdbool.h>

#include "nuttx/irq.h"

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_get_tcb
 *
 * Description:
 *   Given a task ID, obtain a valid TCB and increment the corresponding
 *   reference count to prevent it from being released.
 *
 ****************************************************************************/

FAR struct tcb_s *nxsched_get_tcb(pid_t pid)
{
  FAR struct tcb_s *ret = NULL;
  irqstate_t flags;
  int hash_ndx;

  flags = enter_critical_section();

  if (g_pidhash != NULL && pid >= 0)
    {
      /* The test and the return setup should be atomic.  This still does
       * not provide proper protection if the recipient of the TCB does not
       * also protect against the task associated with the TCB from
       * terminating asynchronously.
       */

      /* Get the hash_ndx associated with the pid */

      hash_ndx = PIDHASH(pid);

      /* Verify that the correct TCB was found. */

      if (g_pidhash[hash_ndx] != NULL && pid == g_pidhash[hash_ndx]->pid)
        {
          /* Return the TCB associated with this pid (if any) */

          ret = g_pidhash[hash_ndx];
        }
    }

  if (ret && ret != running_task())
    {
      if (!up_interrupt_context())
        {
          /* If we are in the thread context, after obtaining a reference to
           * another task, we may not be able to release this reference
           * immediately. The purpose of refs is also to prevent the
           * situation where this_task is killed and thus unable to release
           * the references to other tasks. We need to record the total
           * number of references that this_task makes to other tasks.
           */

          atomic_add(&this_task()->holds, 1);
        }

      atomic_add(&ret->refs, 1);
    }

  leave_critical_section(flags);

  return ret;
}

/****************************************************************************
 * Name: nxsched_put_tcb
 *
 * Description:
 *   Drop a reference taken with nxsched_get_tcb(); each successful get
 *   must be paired with a put.
 *
 ****************************************************************************/

void nxsched_put_tcb(FAR struct tcb_s *tcb)
{
  FAR struct tcb_s *rtcb;
#ifdef CONFIG_SMP
  irqstate_t flags;

  flags = up_irq_save();
  rtcb = running_task();
  up_irq_restore(flags);
#else
  rtcb = running_task();
#endif

  if (!tcb || tcb == rtcb)
    {
      return;
    }

  DEBUGASSERT(atomic_read(&tcb->refs) > 0);

  /* tcb may in EXIT_PROCESSING */

  if (atomic_sub(&tcb->refs, 1) == 1)
    {
      nxsem_post(&tcb->exit_sem);
    }

  if (up_interrupt_context())
    {
      return;
    }

  tcb = this_task();

  DEBUGASSERT(atomic_read(&tcb->holds) > 0);

  /* this_task may be killed and in KILL_PROCESSING: nxtask_terminate()
   * sets the flag before it reads holds, so order our decrement before
   * the flag read.
   */

  bool last = atomic_sub(&tcb->holds, 1) == 1;

  SMP_MB();

  if (last &&
      (atomic_read(&tcb->shared_flags) &
       (TCB_FLAG_KILL_PROCESSING | TCB_FLAG_EXIT_PROCESSING)) != 0)
    {
      /* Wake nxtask_terminate() or nxtask_restart() waiting for the
       * references to be dropped.
       */

      nxsem_post(&tcb->exit_sem);

      /* A killed task stops here: if it kept running it could take the
       * TCB again and be killed again.  A restarted task must keep
       * running until it is reset.
       */

      if ((atomic_read(&tcb->shared_flags) &
           TCB_FLAG_KILL_PROCESSING) != 0)
        {
          for (; ; )
            {
              usleep(1000);
            }
        }
    }
}


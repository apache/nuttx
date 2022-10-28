/****************************************************************************
 * sched/paging/pg_miss.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/page.h>
#include <nuttx/signal.h>

#ifdef CONFIG_PAGING

#include "sched/sched.h"
#include "paging/paging.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pg_miss
 *
 * Description:
 *   This function is called from architecture-specific memory segmentation
 *   fault handling logic.  This function will perform the following
 *   operations:
 *
 *   1) Sanity checking.
 *      - ASSERT if the currently executing task is the page fill worker
 *        thread.  The page fill worker thread is how the page fault
 *        is resolved and all logic associated with the page fill worker
 *        must be "locked" and always present in memory.
 *      - ASSERT if an interrupt was executing at the time of the exception.
 *   2) Block the currently executing task.
 *      - Call up_block_task() to block the task at the head of the ready-
 *        to-run list.  This should cause an interrupt level context switch
 *        to the next highest priority task.
 *      - The blocked task will be marked with state TSTATE_WAIT_PAGEFILL
 *        and will be retained in the g_waitingforfill prioritized task
 *        list.
 *   3) Boost the page fill worker thread priority.
 *      - Check the priority of the task at the head of the g_waitingforfill
 *        list.  If the priority of that task is higher than the current
 *        priority of the page fill worker thread, then boost the priority
 *        of the page fill worker thread to that priority.
 *   4) Signal the page fill worker thread.
 *      - Is there a page fill pending?  If not then signal the worker
 *        thread to start working on the queued page fill requests.
 *
 * Input Parameters:
 *   None - The head of the ready-to-run list is assumed to be task that
 *   caused the exception.
 *
 * Returned Value:
 *   None - Either this function function succeeds or an assertion occurs.
 *
 * Assumptions:
 *   - It is assumed that this function is called from the level of an
 *     exception handler and that all interrupts are disabled.
 *   - It is assumed that currently executing task (the one at the head of
 *     the ready-to-run list) is the one that cause the fault.  This will
 *     always be true unless the page fault occurred in an interrupt handler.
 *     Interrupt handling logic must always be present and "locked" into
 *     memory.
 *   - As mentioned above, the task causing the page fault must not be the
 *     page fill worker thread because that is the only way to complete the
 *     page fill.
 *
 * NOTES:
 *   1. One way to accomplish this would be a two pass link phase:
 *      - In the first phase, create a partially linked objected containing
 *        all interrupt/exception handling logic, the page fill worker thread
 *        plus all parts of the IDLE thread (which must always be available
 *        for execution).
 *      - All of the .text and .rodata sections of this partial link should
 *        be collected into a single section.
 *      - The second link would link the partially linked object along with
 *        the remaining object to produce the final binary.  The linker
 *        script should position the "special" section so that it lies
 *        in a reserved, "non-swappable" region.
 *
 ****************************************************************************/

void pg_miss(void)
{
  FAR struct tcb_s *ftcb = this_task();
  FAR struct tcb_s *wtcb;

  /* Sanity checking
   *
   * ASSERT if the currently executing task is the page fill worker thread.
   * The page fill worker thread is how the page fault is resolved and
   * all logic associated with the page fill worker must be "locked" and
   * always present in memory.
   */

  pginfo("Blocking TCB: %p PID: %d\n", ftcb, ftcb->pid);
  DEBUGASSERT(g_pgworker != ftcb->pid);

  /* Block the currently executing task
   * - Call up_block_task() to block the task at the head of the ready-
   *   to-run list.  This should cause an interrupt level context switch
   *   to the next highest priority task.
   * - The blocked task will be marked with state TSTATE_WAIT_PAGEFILL
   *   and will be retained in the g_waitingforfill prioritized task list.
   *
   * Need to firstly check that this is not the idle task,descheduling
   * that isn't going to end well.
   */

  DEBUGASSERT(!is_idle_task(ftcb));
  up_block_task(ftcb, TSTATE_WAIT_PAGEFILL);

  /* Boost the page fill worker thread priority.
   * - Check the priority of the task at the head of the g_waitingforfill
   *   list.  If the priority of that task is higher than the current
   *   priority of the page fill worker thread, then boost the priority
   *   of the page fill worker thread to that priority.
   */

  wtcb = nxsched_get_tcb(g_pgworker);
  DEBUGASSERT(wtcb != NULL);

  if (wtcb->sched_priority < ftcb->sched_priority)
    {
      /* Reprioritize the page fill worker thread */

      pginfo("New worker priority. %d->%d\n",
             wtcb->sched_priority, ftcb->sched_priority);
      nxsched_set_priority(wtcb, ftcb->sched_priority);
    }

  /* Signal the page fill worker thread.
   * - Is there a page fill pending?  If not then signal the worker
   *   thread to start working on the queued page fill requests.
   */

  if (!g_pftcb)
    {
      pginfo("Signaling worker. PID: %d\n", g_pgworker);
      nxsig_kill(g_pgworker, SIGWORK);
    }
}

#endif /* CONFIG_PAGING */

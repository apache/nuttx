/****************************************************************************
 * sched/sched_removereadytorun.c
 *
 *   Copyright (C) 2007-2009, 2012, 2016-2017 Gregory Nutt.
 *   All rights reserved.
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

#include <stdbool.h>
#include <queue.h>
#include <assert.h>
#include <nuttx/sched_note.h>

#include "irq/irq.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_remove_readytorun
 *
 * Description:
 *   This function removes a TCB from the ready to run list.
 *
 * Input Parameters:
 *   rtcb - Points to the TCB that is ready-to-run
 *
 * Returned Value:
 *   true if the currently active task (the head of the ready-to-run list)
 *     has changed.
 *
 * Assumptions:
 * - The caller has established a critical section before calling this
 *   function (calling sched_lock() first is NOT a good idea -- use
 *   enter_critical_section()).
 * - The caller handles the condition that occurs if the head of the
 *   ready-to-run list is changed.
 *
 ****************************************************************************/

#ifndef CONFIG_SMP
bool nxsched_remove_readytorun(FAR struct tcb_s *rtcb)
{
  bool doswitch = false;

  /* Check if the TCB to be removed is at the head of the ready to run list.
   * There is only one list, g_readytorun, and it always contains the
   * currently running task.  If we are removing the head of this list,
   * then we are removing the currently active task.
   */

  if (rtcb->blink == NULL)
    {
      /* There must always be at least one task in the list (the IDLE task)
       * after the TCB being removed.
       */

      FAR struct tcb_s *nxttcb = (FAR struct tcb_s *)rtcb->flink;
      DEBUGASSERT(nxttcb != NULL);

      nxttcb->task_state = TSTATE_TASK_RUNNING;
      doswitch = true;
    }

  /* Remove the TCB from the ready-to-run list.  In the non-SMP case, this
   * is always the g_readytorun list.
   */

  dq_rem((FAR dq_entry_t *)rtcb, (FAR dq_queue_t *)&g_readytorun);

  /* Since the TCB is not in any list, it is now invalid */

  rtcb->task_state = TSTATE_TASK_INVALID;
  return doswitch;
}
#endif /* !CONFIG_SMP */

/****************************************************************************
 * Name: nxsched_remove_readytorun
 *
 * Description:
 *   This function removes a TCB from the ready to run list.
 *
 * Input Parameters:
 *   rtcb - Points to the TCB that is ready-to-run
 *
 * Returned Value:
 *   true if the currently active task (the head of the ready-to-run list)
 *     has changed.
 *
 * Assumptions:
 * - The caller has established a critical section before calling this
 *   function (calling sched_lock() first is NOT a good idea -- use
 *   enter_critical_section()).
 * - The caller handles the condition that occurs if the head of the
 *   ready-to-run list is changed.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
bool nxsched_remove_readytorun(FAR struct tcb_s *rtcb)
{
  FAR dq_queue_t *tasklist;
  bool doswitch = false;
  int cpu;

  /* Lock the tasklists before accessing */

  irqstate_t lock = nxsched_lock_tasklist();

  /* Which CPU (if any) is the task running on?  Which task list holds the
   * TCB?
   */

  cpu      = rtcb->cpu;
  tasklist = TLIST_HEAD(rtcb->task_state, cpu);

  /* Check if the TCB to be removed is at the head of a ready-to-run list.
   * For the case of SMP, there are two lists involved:  (1) the
   * g_readytorun list that holds non-running tasks that have not been
   * assigned to a CPU, and (2) and the g_assignedtasks[] lists which hold
   * tasks assigned a CPU, including the task that is currently running on
   * that CPU.  Only this latter list contains the currently active task
   * only only removing the head of that list can result in a context
   * switch.
   *
   * rtcb->blink == NULL will tell us if the TCB is at the head of the
   * ready-to-run list and, hence, a candidate for the new running task.
   *
   * If so, then the tasklist RUNNABLE attribute will inform us if the list
   * holds the currently executing task and, hence, if a context switch
   * should occur.
   */

  if (rtcb->blink == NULL && TLIST_ISRUNNABLE(rtcb->task_state))
    {
      FAR struct tcb_s *nxttcb;
      FAR struct tcb_s *rtrtcb = NULL;
      int me;

      /* There must always be at least one task in the list (the IDLE task)
       * after the TCB being removed.
       */

      nxttcb = (FAR struct tcb_s *)rtcb->flink;
      DEBUGASSERT(nxttcb != NULL);

      /* If we are modifying the head of some assigned task list other than
       * our own, we will need to stop that CPU.
       */

      me = this_cpu();
      if (cpu != me)
        {
          nxsched_unlock_tasklist(lock);
          DEBUGVERIFY(up_cpu_pause(cpu));
          lock = nxsched_lock_tasklist();
        }

      /* The task is running but the CPU that it was running on has been
       * paused.  We can now safely remove its TCB from the ready-to-run
       * task list.  In the SMP case this may be either the g_readytorun()
       * or the g_assignedtasks[cpu] list.
       */

      dq_rem((FAR dq_entry_t *)rtcb, tasklist);

      /* Which task will go at the head of the list?  It will be either the
       * next tcb in the assigned task list (nxttcb) or a TCB in the
       * g_readytorun list.  We can only select a task from that list if
       * the affinity mask includes the current CPU.
       *
       * If pre-emption is locked or another CPU is in a critical section,
       * then use the 'nxttcb' which will probably be the IDLE thread.
       * REVISIT: What if it is not the IDLE thread?
       */

      if (!nxsched_islocked_global() && !irq_cpu_locked(me))
        {
          /* Search for the highest priority task that can run on this
           * CPU.
           */

          for (rtrtcb = (FAR struct tcb_s *)g_readytorun.head;
               rtrtcb != NULL && !CPU_ISSET(cpu, &rtrtcb->affinity);
               rtrtcb = (FAR struct tcb_s *)rtrtcb->flink);
        }

      /* Did we find a task in the g_readytorun list?  Which task should
       * we use?  We decide strictly by the priority of the two tasks:
       * Either (1) the task currently at the head of the
       * g_assignedtasks[cpu] list (nexttcb) or (2) the highest priority
       * task from the g_readytorun list with matching affinity (rtrtcb).
       */

      if (rtrtcb != NULL && rtrtcb->sched_priority >= nxttcb->sched_priority)
        {
          FAR struct tcb_s *tmptcb;

          /* The TCB at the head of the ready to run list has the higher
           * priority.  Remove that task from the head of the g_readytorun
           * list and add to the head of the g_assignedtasks[cpu] list.
           */

          tmptcb = (FAR struct tcb_s *)
            dq_remfirst((FAR dq_queue_t *)&g_readytorun);

          dq_addfirst((FAR dq_entry_t *)tmptcb, tasklist);

          tmptcb->cpu = cpu;
          nxttcb = tmptcb;
        }

      /* Will pre-emption be disabled after the switch?  If the lockcount is
       * greater than zero, then this task/this CPU holds the scheduler lock.
       */

      if (nxttcb->lockcount > 0)
        {
          /* Yes... make sure that scheduling logic knows about this */

          spin_setbit(&g_cpu_lockset, cpu, &g_cpu_locksetlock,
                      &g_cpu_schedlock);
        }
      else
        {
          /* No.. we may need to perform release our hold on the lock. */

          spin_clrbit(&g_cpu_lockset, cpu, &g_cpu_locksetlock,
                      &g_cpu_schedlock);
        }

      /* Adjust global IRQ controls.  If irqcount is greater than zero,
       * then this task/this CPU holds the IRQ lock
       */

      if (nxttcb->irqcount > 0)
        {
          /* Yes... make sure that scheduling logic on other CPUs knows
           * that we hold the IRQ lock.
           */

          spin_setbit(&g_cpu_irqset, cpu, &g_cpu_irqsetlock,
                      &g_cpu_irqlock);
        }

      /* No.. This CPU will be relinquishing the lock.  But this works
       * differently if we are performing a context switch from an
       * interrupt handler and the interrupt handler has established
       * a critical section.  We can detect this case when
       * g_cpu_nestcount[me] > 0.
       */

      else if (g_cpu_nestcount[me] <= 0)
        {
          /* Do nothing here
           * NOTE: spin_clrbit() will be done in sched_resumescheduler()
           */
        }

      /* Sanity check.  g_cpu_netcount should be greater than zero
       * only while we are within the critical section and within
       * an interrupt handler.  If we are not in an interrupt handler
       * then there is a problem; perhaps some logic previously
       * called enter_critical_section() with no matching call to
       * leave_critical_section(), leaving the non-zero count.
       */

      else
        {
          DEBUGASSERT(up_interrupt_context());
        }

      nxttcb->task_state = TSTATE_TASK_RUNNING;

      /* All done, restart the other CPU (if it was paused). */

      doswitch = true;
      if (cpu != me)
        {
          /* In this we will not want to report a context switch to this
           * CPU.  Only the other CPU is affected.
           */

          DEBUGVERIFY(up_cpu_resume(cpu));
          doswitch = false;
        }
    }
  else
    {
      /* The task is not running.  Just remove its TCB from the ready-to-run
       * list.  In the SMP case this may be either the g_readytorun() or the
       * g_assignedtasks[cpu] list.
       */

      dq_rem((FAR dq_entry_t *)rtcb, tasklist);
    }

  /* Since the TCB is no longer in any list, it is now invalid */

  rtcb->task_state = TSTATE_TASK_INVALID;

  /* Unlock the tasklists */

  nxsched_unlock_tasklist(lock);
  return doswitch;
}
#endif /* CONFIG_SMP */

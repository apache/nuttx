/****************************************************************************
 * sched/sched/sched_addreadytorun.c
 *
 *   Copyright (C) 2007-2009, 2014, 2016-2018 Gregory Nutt. All rights
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

#include <stdbool.h>
#include <queue.h>
#include <assert.h>

#include "irq/irq.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_addreadytorun
 *
 * Description:
 *   This function adds a TCB to the ready to run list.  If the currently
 *   active task has preemption disabled and the new TCB would cause this
 *   task to be pre-empted, the new task is added to the g_pendingtasks list
 *   instead.  The pending tasks will be made ready-to-run when preemption is
 *   unlocked.
 *
 * Input Parameters:
 *   btcb - Points to the blocked TCB that is ready-to-run
 *
 * Returned Value:
 *   true if the currently active task (the head of the ready-to-run list)
 *   has changed.
 *
 * Assumptions:
 * - The caller has established a critical section before calling this
 *   function (calling sched_lock() first is NOT a good idea -- use
 *   enter_critical_section()).
 * - The caller has already removed the input rtcb from whatever list it
 *   was in.
 * - The caller handles the condition that occurs if the head of the
 *   ready-to-run list is changed.
 *
 ****************************************************************************/

#ifndef CONFIG_SMP
bool sched_addreadytorun(FAR struct tcb_s *btcb)
{
  FAR struct tcb_s *rtcb = this_task();
  bool ret;

  /* Check if pre-emption is disabled for the current running task and if
   * the new ready-to-run task would cause the current running task to be
   * pre-empted.  NOTE that IRQs disabled implies that pre-emption is
   * also disabled.
   */

  if (rtcb->lockcount > 0 && rtcb->sched_priority < btcb->sched_priority)
    {
      /* Yes.  Preemption would occur!  Add the new ready-to-run task to the
       * g_pendingtasks task list for now.
       */

      sched_addprioritized(btcb, (FAR dq_queue_t *)&g_pendingtasks);
      btcb->task_state = TSTATE_TASK_PENDING;
      ret = false;
    }

  /* Otherwise, add the new task to the ready-to-run task list */

  else if (sched_addprioritized(btcb, (FAR dq_queue_t *)&g_readytorun))
    {
      /* The new btcb was added at the head of the ready-to-run list.  It
       * is now the new active task!
       */

      DEBUGASSERT(rtcb->lockcount == 0 && btcb->flink != NULL);

      btcb->task_state = TSTATE_TASK_RUNNING;
      btcb->flink->task_state = TSTATE_TASK_READYTORUN;
      ret = true;
    }
  else
    {
      /* The new btcb was added in the middle of the ready-to-run list */

      btcb->task_state = TSTATE_TASK_READYTORUN;
      ret = false;
    }

  return ret;
}
#endif /* !CONFIG_SMP */

/****************************************************************************
 * Name:  sched_addreadytorun
 *
 * Description:
 *   This function adds a TCB to one of the ready to run lists.  That might
 *   be:
 *
 *   1. The g_readytorun list if the task is ready-to-run but not running
 *      and not assigned to a CPU.
 *   2. The g_assignedtask[cpu] list if the task is running or if has been
 *      assigned to a CPU.
 *
 *   If the currently active task has preemption disabled and the new TCB
 *   would cause this task to be pre-empted, the new task is added to the
 *   g_pendingtasks list instead.  The pending tasks will be made
 *   ready-to-run when preemption isunlocked.
 *
 * Input Parameters:
 *   btcb - Points to the blocked TCB that is ready-to-run
 *
 * Returned Value:
 *   true if the currently active task (the head of the ready-to-run list)
 *   has changed.
 *
 * Assumptions:
 * - The caller has established a critical section before calling this
 *   function (calling sched_lock() first is NOT a good idea -- use
 *   enter_critical_section()).
 * - The caller has already removed the input rtcb from whatever list it
 *   was in.
 * - The caller handles the condition that occurs if the head of the
 *   ready-to-run list is changed.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
bool sched_addreadytorun(FAR struct tcb_s *btcb)
{
  FAR struct tcb_s *rtcb;
  FAR dq_queue_t *tasklist;
  bool switched;
  bool doswitch;
  int task_state;
  int cpu;
  int me;

  /* Lock the tasklists before accessing */

  irqstate_t lock = sched_tasklist_lock();

  /* Check if the blocked TCB is locked to this CPU */

  if ((btcb->flags & TCB_FLAG_CPU_LOCKED) != 0)
    {
      /* Yes.. that that is the CPU we must use */

      cpu  = btcb->cpu;
    }
  else
    {
      /* Otherwise, find the CPU that is executing the lowest priority task
       * (possibly its IDLE task).
       */

      cpu = sched_cpu_select(btcb->affinity);
    }

  /* Get the task currently running on the CPU (may be the IDLE task) */

  rtcb = (FAR struct tcb_s *)g_assignedtasks[cpu].head;

  /* Determine the desired new task state.  First, if the new task priority
   * is higher then the priority of the lowest priority, running task, then
   * the new task will be running and a context switch switch will be required.
   */

  if (rtcb->sched_priority < btcb->sched_priority)
    {
      task_state = TSTATE_TASK_RUNNING;
    }

  /* If it will not be running, but is locked to a CPU, then it will be in
   * the assigned state.
   */

  else if ((btcb->flags & TCB_FLAG_CPU_LOCKED) != 0)
    {
      task_state = TSTATE_TASK_ASSIGNED;
      cpu = btcb->cpu;
    }

  /* Otherwise, it will be ready-to-run, but not not yet running */

  else
    {
      task_state = TSTATE_TASK_READYTORUN;
      cpu = 0;  /* CPU does not matter */
    }

  /* If the selected state is TSTATE_TASK_RUNNING, then we would like to
   * start running the task.  Be we cannot do that if pre-emption is
   * disabled.  If the selected state is TSTATE_TASK_READYTORUN, then it
   * should also go to the pending task list so that it will have a chance
   * to be restarted when the scheduler is unlocked.
   *
   * There is an interaction here with IRQ locking.  Even if the pre-
   * emption is enabled, tasks will be forced to pend if the IRQ lock
   * is also set UNLESS the CPU starting the thread is also the holder of
   * the IRQ lock.  irq_cpu_locked() performs an atomic check for that
   * situation.
   */

  me = this_cpu();
  if ((sched_islocked_global() || irq_cpu_locked(me)) &&
      task_state != TSTATE_TASK_ASSIGNED)
    {
      /* Add the new ready-to-run task to the g_pendingtasks task list for
       * now.
       */

      sched_addprioritized(btcb, (FAR dq_queue_t *)&g_pendingtasks);
      btcb->task_state = TSTATE_TASK_PENDING;
      doswitch = false;
    }
  else if (task_state == TSTATE_TASK_READYTORUN)
    {
      /* The new btcb was added either (1) in the middle of the assigned
       * task list (the btcb->cpu field is already valid) or (2) was
       * added to the ready-to-run list (the btcb->cpu field does not
       * matter).  Either way, it won't be running.
       *
       * Add the task to the ready-to-run (but not running) task list
       */

      (void)sched_addprioritized(btcb, (FAR dq_queue_t *)&g_readytorun);

      btcb->task_state = TSTATE_TASK_READYTORUN;
      doswitch         = false;
    }
  else /* (task_state == TSTATE_TASK_ASSIGNED || task_state == TSTATE_TASK_RUNNING) */
    {
      /* If we are modifying some assigned task list other than our own, we
       * will need to stop that CPU.
       */

      if (cpu != me)
        {
          sched_tasklist_unlock(lock);
          DEBUGVERIFY(up_cpu_pause(cpu));
          lock = sched_tasklist_lock();
        }

      /* Add the task to the list corresponding to the selected state
       * and check if a context switch will occur
       */

      tasklist = (FAR dq_queue_t *)&g_assignedtasks[cpu];
      switched = sched_addprioritized(btcb, tasklist);

      /* If the selected task list was the g_assignedtasks[] list and if the
       * new tasks is the highest priority (RUNNING) task, then a context
       * switch will occur.
       */

      if (switched)
        {
          FAR struct tcb_s *next;

          /* The new btcb was added at the head of the ready-to-run list.  It
           * is now the new active task!
           */

          /* Assign the CPU and set the running state */

          DEBUGASSERT(task_state == TSTATE_TASK_RUNNING);

          btcb->cpu        = cpu;
          btcb->task_state = TSTATE_TASK_RUNNING;

          /* Adjust global pre-emption controls.  If the lockcount is
           * greater than zero, then this task/this CPU holds the scheduler
           * lock.
           */

          if (btcb->lockcount > 0)
            {
              spin_setbit(&g_cpu_lockset, cpu, &g_cpu_locksetlock,
                          &g_cpu_schedlock);
            }
          else
            {
              spin_clrbit(&g_cpu_lockset, cpu, &g_cpu_locksetlock,
                          &g_cpu_schedlock);
            }

          /* Adjust global IRQ controls.  If irqcount is greater than zero,
           * then this task/this CPU holds the IRQ lock
           */

          if (btcb->irqcount > 0)
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

          /* If the following task is not locked to this CPU, then it must
           * be moved to the g_readytorun list.  Since it cannot be at the
           * head of the list, we can do this without invoking any heavy
           * lifting machinery.
           */

          DEBUGASSERT(btcb->flink != NULL);
          next = (FAR struct tcb_s *)btcb->flink;

          if ((next->flags & TCB_FLAG_CPU_LOCKED) != 0)
            {
              DEBUGASSERT(next->cpu == cpu);
              next->task_state = TSTATE_TASK_ASSIGNED;
            }
          else
            {
              /* Remove the task from the assigned task list */

              dq_rem((FAR dq_entry_t *)next, tasklist);

              /* Add the task to the g_readytorun or to the g_pendingtasks
               * list.  NOTE: That the above operations may cause the
               * scheduler to become locked.  It may be assigned to a
               * different CPU the next time that it runs.
               */

              if (sched_islocked_global())
                {
                  next->task_state = TSTATE_TASK_PENDING;
                  tasklist         = (FAR dq_queue_t *)&g_pendingtasks;
                }
              else
                {
                  next->task_state = TSTATE_TASK_READYTORUN;
                  tasklist         = (FAR dq_queue_t *)&g_readytorun;
                }

              (void)sched_addprioritized(next, tasklist);
            }

          doswitch = true;
        }
      else
        {
          /* No context switch.  Assign the CPU and set the assigned state.
           *
           * REVISIT: I have seen this assertion fire.  Apparently another
           * CPU may add another, higher priority task to the same
           * g_assignedtasks[] list sometime after sched_cpu_select() was
           * called above, leaving this TCB in the wrong task list if task_state
           * is TSTATE_TASK_ASSIGNED).
           */

          DEBUGASSERT(task_state == TSTATE_TASK_ASSIGNED);

          btcb->cpu        = cpu;
          btcb->task_state = TSTATE_TASK_ASSIGNED;
        }

      /* All done, restart the other CPU (if it was paused). */

      if (cpu != me)
        {
          DEBUGVERIFY(up_cpu_resume(cpu));
          doswitch = false;
        }
    }

  /* Unlock the tasklists */

  sched_tasklist_unlock(lock);
  return doswitch;
}

#endif /* CONFIG_SMP */

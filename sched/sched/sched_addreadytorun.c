/****************************************************************************
 * sched/sched/sched_addreadytorun.c
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
#include <assert.h>

#include <nuttx/queue.h>

#include "irq/irq.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_add_readytorun
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
bool nxsched_add_readytorun(FAR struct tcb_s *btcb)
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

      nxsched_add_prioritized(btcb, &g_pendingtasks);
      btcb->task_state = TSTATE_TASK_PENDING;
      ret = false;
    }

  /* Otherwise, add the new task to the ready-to-run task list */

  else if (nxsched_add_prioritized(btcb, &g_readytorun))
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
 * Name:  nxsched_add_readytorun
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
 *   ready-to-run list has changed.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
bool nxsched_add_readytorun(FAR struct tcb_s *btcb)
{
  FAR struct tcb_s *rtcb;
  FAR dq_queue_t *tasklist;
  bool switched;
  bool doswitch;
  int task_state;
  int cpu;
  int me;

  /* Check if the blocked TCB is locked to this CPU */

  if ((btcb->flags & TCB_FLAG_CPU_LOCKED) != 0)
    {
      /* Yes.. that is the CPU we must use */

      cpu  = btcb->cpu;
    }
  else
    {
      /* Otherwise, find the CPU that is executing the lowest priority task
       * (possibly its IDLE task).
       */

      cpu = nxsched_select_cpu(btcb->affinity);
    }

  /* Get the task currently running on the CPU (may be the IDLE task) */

  rtcb = (FAR struct tcb_s *)g_assignedtasks[cpu].head;

  /* Determine the desired new task state.  First, if the new task priority
   * is higher then the priority of the lowest priority, running task, then
   * the new task will be running and a context switch switch will be
   * required.
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
  if ((nxsched_islocked_global() || irq_cpu_locked(me)) &&
      task_state != TSTATE_TASK_ASSIGNED)
    {
      /* Add the new ready-to-run task to the g_pendingtasks task list for
       * now.
       */

      nxsched_add_prioritized(btcb, &g_pendingtasks);
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

      nxsched_add_prioritized(btcb, &g_readytorun);

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
          DEBUGVERIFY(up_cpu_pause(cpu));
        }

      /* Add the task to the list corresponding to the selected state
       * and check if a context switch will occur
       */

      tasklist = &g_assignedtasks[cpu];
      switched = nxsched_add_prioritized(btcb, tasklist);

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

          /* NOTE: If the task runs on another CPU(cpu), adjusting global IRQ
           * controls will be done in the pause handler on the new CPU(cpu).
           * If the task is scheduled on this CPU(me), do nothing because
           * this CPU already has a critical section
           */

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

              if (nxsched_islocked_global())
                {
                  next->task_state = TSTATE_TASK_PENDING;
                  tasklist         = &g_pendingtasks;
                }
              else
                {
                  next->task_state = TSTATE_TASK_READYTORUN;
                  tasklist         = &g_readytorun;
                }

              nxsched_add_prioritized(next, tasklist);
            }

          doswitch = true;
        }
      else
        {
          /* No context switch.  Assign the CPU and set the assigned state.
           *
           * REVISIT: I have seen this assertion fire.  Apparently another
           * CPU may add another, higher priority task to the same
           * g_assignedtasks[] list sometime after nxsched_select_cpu() was
           * called above, leaving this TCB in the wrong task list if
           * task_state is TSTATE_TASK_ASSIGNED).
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

  return doswitch;
}

#endif /* CONFIG_SMP */

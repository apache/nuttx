/****************************************************************************
 * sched/sched/sched_lock.c
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

#include <sys/types.h>
#include <sched.h>
#include <assert.h>

#include <arch/irq.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Pre-emption is disabled via the interface sched_lock(). sched_lock()
 * works by preventing context switches from the currently executing tasks.
 * This prevents other tasks from running (without disabling interrupts) and
 * gives the currently executing task exclusive access to the (single) CPU
 * resources. Thus, sched_lock() and its companion, sched_unlock(), are
 * used to implement some critical sections.
 *
 * In the single CPU case, pre-emption is disabled using a simple lockcount
 * in the TCB. When the scheduling is locked, the lockcount is incremented;
 * when the scheduler is unlocked, the lockcount is decremented. If the
 * lockcount for the task at the head of the g_readytorun list has a
 * lockcount > 0, then pre-emption is disabled.
 *
 * No special protection is required since only the executing task can
 * modify its lockcount.
 */

#ifdef CONFIG_SMP
/* In the multiple CPU, SMP case, disabling context switches will not give a
 * task exclusive access to the (multiple) CPU resources (at least without
 * stopping the other CPUs): Even though pre-emption is disabled, other
 * threads will still be executing on the other CPUS.
 *
 * There are additional rules for this multi-CPU case:
 *
 * 1. There is a global lock count 'g_cpu_lockset' that includes a bit for
 *    each CPU: If the bit is '1', then the corresponding CPU has the
 *    scheduler locked; if '0', then the CPU does not have the scheduler
 *    locked.
 * 2. Scheduling logic would set the bit associated with the cpu in
 *    'g_cpu_lockset' when the TCB at the head of the g_assignedtasks[cpu]
 *    list transitions has 'lockcount' > 0. This might happen when
 *    sched_lock() is called, or after a context switch that changes the
 *    TCB at the head of the g_assignedtasks[cpu] list.
 * 3. Similarly, the cpu bit in the global 'g_cpu_lockset' would be cleared
 *    when the TCB at the head of the g_assignedtasks[cpu] list has
 *    'lockcount' == 0. This might happen when sched_unlock() is called, or
 *    after a context switch that changes the TCB at the head of the
 *    g_assignedtasks[cpu] list.
 * 4. Modification of the global 'g_cpu_lockset' must be protected by a
 *    spinlock, 'g_cpu_schedlock'. That spinlock would be taken when
 *    sched_lock() is called, and released when sched_unlock() is called.
 *    This assures that the scheduler does enforce the critical section.
 *    NOTE: Because of this spinlock, there should never be more than one
 *    bit set in 'g_cpu_lockset'; attempts to set additional bits should
 *    be cause the CPU to block on the spinlock.  However, additional bits
 *    could get set in 'g_cpu_lockset' due to the context switches on the
 *    various CPUs.
 * 5. Each the time the head of a g_assignedtasks[] list changes and the
 *    scheduler modifies 'g_cpu_lockset', it must also set 'g_cpu_schedlock'
 *    depending on the new state of 'g_cpu_lockset'.
 * 5. Logic that currently uses the currently running tasks lockcount
 *    instead uses the global 'g_cpu_schedlock'. A value of SP_UNLOCKED
 *    means that no CPU has pre-emption disabled; SP_LOCKED means that at
 *    least one CPU has pre-emption disabled.
 */

volatile spinlock_t g_cpu_schedlock SP_SECTION = SP_UNLOCKED;

/* Used to keep track of which CPU(s) hold the IRQ lock. */

volatile spinlock_t g_cpu_locksetlock SP_SECTION;
volatile cpu_set_t g_cpu_lockset SP_SECTION;

#endif /* CONFIG_SMP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_lock
 *
 * Description:
 *   This function disables context switching by disabling addition of
 *   new tasks to the g_readytorun task list.  The task that calls this
 *   function will be the only task that is allowed to run until it
 *   either calls  sched_unlock() (the appropriate number of times) or
 *   until it blocks itself.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SMP

int sched_lock(void)
{
  FAR struct tcb_s *rtcb;
  irqstate_t flags;
  int cpu;

  /* If the CPU supports suppression of interprocessor interrupts, then
   * simple disabling interrupts will provide sufficient protection for
   * the following operation.
   */

  flags = up_irq_save();

  cpu  = this_cpu();
  rtcb = current_task(cpu);

  /* Check for some special cases:  (1) rtcb may be NULL only during early
   * boot-up phases, and (2) sched_lock() should have no effect if called
   * from the interrupt level.
   */

  if (rtcb == NULL || up_interrupt_context())
    {
      up_irq_restore(flags);
    }
  else
    {
      /* Catch attempts to increment the lockcount beyond the range of the
       * integer type.
       */

      DEBUGASSERT(rtcb->lockcount < MAX_LOCK_COUNT);

      /* We must hold the lock on this CPU before we increment the lockcount
       * for the first time. Holding the lock is sufficient to lockout
       * context switching.
       */

      if (rtcb->lockcount == 0)
        {
          /* We don't have the scheduler locked.  But logic running on a
           * different CPU may have the scheduler locked.  It is not
           * possible for some other task on this CPU to have the scheduler
           * locked (or we would not be executing!).
           */

          spin_setbit(&g_cpu_lockset, this_cpu(), &g_cpu_locksetlock,
                      &g_cpu_schedlock);
        }
      else
        {
          /* If this thread already has the scheduler locked, then
           * g_cpu_schedlock() should indicate that the scheduler is locked
           * and g_cpu_lockset should include the bit setting for this CPU.
           */

          DEBUGASSERT(g_cpu_schedlock == SP_LOCKED &&
                      (g_cpu_lockset & (1 << this_cpu())) != 0);
        }

      /* A counter is used to support locking.  This allows nested lock
       * operations on this thread (on any CPU)
       */

      rtcb->lockcount++;

      up_irq_restore(flags);

#if defined(CONFIG_SCHED_INSTRUMENTATION_PREEMPTION) || \
    defined(CONFIG_SCHED_CRITMONITOR)
      /* Check if we just acquired the lock */

      if (rtcb->lockcount == 1)
        {
          /* Note that we have pre-emption locked */

#ifdef CONFIG_SCHED_CRITMONITOR
          nxsched_critmon_preemption(rtcb, true);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
          sched_note_premption(rtcb, true);
#endif
        }
#endif

      /* Move any tasks in the ready-to-run list to the pending task list
       * where they will not be available to run until the scheduler is
       * unlocked and nxsched_merge_pending() is called.
       */

      nxsched_merge_prioritized((FAR dq_queue_t *)&g_readytorun,
                                (FAR dq_queue_t *)&g_pendingtasks,
                                TSTATE_TASK_PENDING);
    }

  return OK;
}

#else /* CONFIG_SMP */

int sched_lock(void)
{
  FAR struct tcb_s *rtcb = this_task();

  /* Check for some special cases:  (1) rtcb may be NULL only during early
   * boot-up phases, and (2) sched_lock() should have no effect if called
   * from the interrupt level.
   */

  if (rtcb != NULL && !up_interrupt_context())
    {
      /* Catch attempts to increment the lockcount beyond the range of the
       * integer type.
       */

      DEBUGASSERT(rtcb->lockcount < MAX_LOCK_COUNT);

      /* A counter is used to support locking.  This allows nested lock
       * operations on this thread (on any CPU)
       */

      rtcb->lockcount++;

#if defined(CONFIG_SCHED_INSTRUMENTATION_PREEMPTION) || \
    defined(CONFIG_SCHED_CRITMONITOR)
      /* Check if we just acquired the lock */

      if (rtcb->lockcount == 1)
        {
          /* Note that we have pre-emption locked */

#ifdef CONFIG_SCHED_CRITMONITOR
          nxsched_critmon_preemption(rtcb, true);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
          sched_note_premption(rtcb, true);
#endif
        }
#endif
    }

  return OK;
}

#endif /* CONFIG_SMP */

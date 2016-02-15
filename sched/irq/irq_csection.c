/****************************************************************************
 * sched/irq/irq_csection.c
 *
 *   Copyright (C) 2007-2008, 2010 Gregory Nutt. All rights reserved.
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

#include <nuttx/sched.h>
#include <nuttx/spinlock.h>
#include <arch/irq.h>

#include "sched/sched.h"
#include "irq/irq.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Public Data
 ****************************************************************************/
/* This is the spinlock that enforces critical sections when interrupts are
 * disabled.
 */

spinlock_t g_cpu_irqlock = SP_UNLOCKED;

/* Used to keep track of which CPU(s) hold the IRQ lock.  There really should
 * only be one.
 */

#if (CONFIG_SMP_NCPUS <= 8)
volatile uint8_t  g_cpu_irqset;
#elif (CONFIG_SMP_NCPUS <= 16)
volatile uint16_t g_cpu_irqset;
#elif (CONFIG_SMP_NCPUS <= 32)
volatile uint32_t g_cpu_irqset;
#else
#  error SMP: Extensions needed to support this number of CPUs
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: enter_critical_section
 *
 * Description:
 *   Take the CPU IRQ lock and disable interrupts on all CPUs.  A thread-
 *   specific counter is increment to indicate that the thread has IRQs
 *   disabled and to support nested calls to enter_critical_section().
 *
 ****************************************************************************/

irqstate_t enter_critical_section(void)
{
  FAR struct tcb_s *rtcb;

  /* Do nothing if called from an interrupt handler */

  if (up_interrupt_context())
    {
      /* The value returned does not matter.  We assume only that it is a
       * scalar here.
       */

      return (irqstate_t)0;
    }

  /* Do we already have interrupts disabled? */

  rtcb = this_task();
  if (rtcb->irqcount > 0)
    {
      /* Yes... make sure that the spinlock is set and increment the IRQ
       * lock count.
       */

      DEBUGASSERT(g_cpu_irqlock == SP_LOCKED && rtcb->irqcount < INT16_MAX);
      rtcb->irqcount++;
    }
  else
    {
      /* NO.. Take the spinlock to get exclusive access and set the lock
       * count to 1.
       *
       * We must avoid that case where a context occurs between taking the
       * g_cpu_irqlock and disabling interrupts.  Also interrupts disables
       * must follow a stacked order.  We cannot other context switches to
       * re-order the enabling/disabling of interrupts.  We can both by
       * locking the scheduler while interrupts are disabled.
       *
       * NOTE: that sched_lock() also uses a spinlock.  We will avoid
       * deadlocks by assuring that the scheduler spinlock is always
       * taken before the IRQ spinlock.
       */

      sched_lock();
      spin_lock(&g_cpu_irqlock);

      rtcb->irqcount = 1;
      g_cpu_irqset  |= (1 << this_cpu());
    }

  /* Then disable interrupts (they may already be disabled, be we need to
   * return valid interrupt status in any event).
   */

  return up_irq_save();
}

/****************************************************************************
 * Name: leave_critical_section
 *
 * Description:
 *   Decrement the IRQ lock count and if it decrements to zero then release
 *   the spinlock.
 *
 ****************************************************************************/

void leave_critical_section(irqstate_t flags)
{
  /* Do nothing if called from an interrupt handler */

  if (!up_interrupt_context())
    {
      FAR struct tcb_s *rtcb = this_task();

      DEBUGASSERT(rtcb->irqcount > 0);

      /* Will we still have interrupts disabled after decrementing the
       * count?
       */

      if (rtcb->irqcount > 1)
        {
          /* Yes... make sure that the spinlock is set */

          DEBUGASSERT(g_cpu_irqlock == SP_LOCKED);
          rtcb->irqcount--;
        }
      else
        {
          /* NO.. Release the spinlock to allow other access.
           *
           * REVISIT: There is a cornercase where multiple CPUs may think
           * they are the holder of the IRQ spinlock.  We will need to disable
           * the scheduler to assure that the following operation is atomic
           * Hmmm... but that could cause a deadlock!  What to do?  Perhaps
           * an array of booleans instead of a bitset?
           */

          g_cpu_irqset  &= ~(1 << this_cpu());
          rtcb->irqcount = 0;
          spin_unlock(g_cpu_irqlock);

          /* And re-enable pre-emption */

          sched_unlock();
        }

      /* Restore the previous interrupt state which may still be interrupts
       * disabled (but we don't have a mechanism to verify that now)
       */

      up_irq_restore(flags);
    }
}

#endif /* CONFIG_SMP */

/****************************************************************************
 * sched/semaphore/spinlock.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <sched.h>
#include <assert.h>

#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>
#include <arch/irq.h>

#include "sched/sched.h"

#ifdef CONFIG_SPINLOCK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMPOSSIBLE_CPU 0xff

/* REVISIT:  What happens if a thread taks a spinlock while running on one
 * CPU, but is suspended, then reassigned to another CPU where it runs and
 * eventually calls spin_unlock().  One solution might be to lock a thread to
 * a CPU if it holds a spinlock.  That would assure that it never runs on
 * any other CPU and avoids such complexities.
 */

#undef CONFIG_SPINLOCK_LOCKDOWN /* Feature not yet available */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spin_initializer
 *
 * Description:
 *   Initialize a re-entrant spinlock object to its initial, unlocked state.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to be initialized.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void spin_initializer(FAR struct spinlock_s *lock)
{
  DEBUGASSERT(lock != NULL);

  lock->sp_lock  = SP_UNLOCKED;
#ifdef CONFIG_SMP
  lock->sp_cpu   = IMPOSSIBLE_CPU;
  lock->sp_count = 0;
#endif
}

/****************************************************************************
 * Name: spin_lock
 *
 * Description:
 *   If this CPU does not already hold the spinlock, then loop until the
 *   spinlock is successfully locked.
 *
 *   This implementation is non-reentrant and is prone to deadlocks in
 *   the case that any logic on the same CPU attempts to take the lock
 *   more than one
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   None.  When the function returns, the spinlock was successfully locked
 *   by this CPU.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

void spin_lock(FAR volatile spinlock_t *lock)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  /* Notify that we are waiting for a spinlock */

  sched_note_spinlock(this_task(), lock);
#endif

  while (up_testset(lock) == SP_LOCKED)
    {
      SP_DSB();
    }

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  /* Notify that we have the spinlock */

  sched_note_spinlocked(this_task(), lock);
#endif
  SP_DMB();
}

/****************************************************************************
 * Name: spin_lock_wo_note
 *
 * Description:
 *   If this CPU does not already hold the spinlock, then loop until the
 *   spinlock is successfully locked.
 *
 *   This implementation is the same as the above spin_lock() except that
 *   it does not perform instrumentation logic.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   None.  When the function returns, the spinlock was successfully locked
 *   by this CPU.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

void spin_lock_wo_note(FAR volatile spinlock_t *lock)
{
  while (up_testset(lock) == SP_LOCKED)
    {
      SP_DSB();
    }

  SP_DMB();
}

/****************************************************************************
 * Name: spin_unlock
 *
 * Description:
 *   Release one count on a non-reentrant spinlock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to unlock.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

#ifdef __SP_UNLOCK_FUNCTION
void spin_unlock(FAR volatile spinlock_t *lock)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  /* Notify that we are unlocking the spinlock */

  sched_note_spinunlock(this_task(), lock);
#endif

  SP_DMB();
  *lock = SP_UNLOCKED;
  SP_DSB();
}
#endif

/****************************************************************************
 * Name: spin_unlock_wo_note
 *
 * Description:
 *   Release one count on a non-reentrant spinlock.
 *
 *   This implementation is the same as the above spin_unlock() except that
 *   it does not perform instrumentation logic.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to unlock.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

void spin_unlock_wo_note(FAR volatile spinlock_t *lock)
{
  SP_DMB();
  *lock = SP_UNLOCKED;
  SP_DSB();
}

/****************************************************************************
 * Name: spin_lockr
 *
 * Description:
 *   If this CPU does not already hold the spinlock, then loop until the
 *   spinlock is successfully locked.
 *
 *   This implementation is re-entrant in the sense that it can called
 *   numerous times from the same CPU without blocking.  Of course,
 *   spin_unlock() must be called the same number of times.  NOTE: the
 *   thread that originallly took the look may be executing on a different
 *   CPU when it unlocks the spinlock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   None.  When the function returns, the spinlock was successfully locked
 *   by this CPU.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

void spin_lockr(FAR struct spinlock_s *lock)
{
#ifdef CONFIG_SMP
  irqstate_t flags;
  uint8_t cpu = this_cpu();

  /* Disable interrupts (all CPUs) */

  flags = up_irq_save();

  /* Do we already hold the lock on this CPU? */

  if (lock->sp_cpu == cpu)
    {
      /* Yes... just increment the number of references we have on the lock */

      lock->sp_count++;
      DEBUGASSERT(lock->sp_lock = SP_LOCKED && lock->sp_count > 0);
    }
  else
    {
#ifdef CONFIG_SPINLOCK_LOCKDOWN
      /* REVISIT:  What happens if this thread is suspended, then reassigned
       * to another CPU where it runs and eventually calls spin_unlock().
       * One solution might be to lock a thread to a CPU if it holds a
       * spinlock.  That would assure that it never runs on any other CPU
       * and avoids such complexities.
       */

#  warning Missing logic
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
      /* Notify that we are waiting for a spinlock */

      sched_note_spinlock(this_task(), &lock->sp_lock);
#endif

      /* Take the lock.  REVISIT:  We should set an indication in the TCB
       * that the thread is spinning.  This might be useful in determining
       * some scheduling actions?
       */

      while (up_testset(&lock->sp_lock) == SP_LOCKED)
        {
          up_irq_restore(flags);
          sched_yield();
          flags = up_irq_save();
          SP_DSB();
        }

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
      /* Notify that we have thespinlock */

      sched_note_spinlocked(this_task(), &lock->sp_lock);
#endif

      SP_DMB();

      /* Take one count on the lock */

      lock->sp_cpu   = cpu;
      lock->sp_count = 1;
    }

  up_irq_restore(flags);

#else /* CONFIG_SMP */

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  /* Notify that we are waiting for a spinlock */

  sched_note_spinlock(this_task(), &lock->sp_lock);
#endif

  /* Take the lock.  REVISIT:  We should set an indication in the TCB that
   * the thread is spinning.  This might be useful in determining some
   * scheduling actions?
   */

  while (up_testset(&lock->sp_lock) == SP_LOCKED)
    {
      sched_yield();
      SP_DSB()
    }

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  /* Notify that we have thespinlock */

  sched_note_spinlocked(this_task(), &lock->sp_lock);
#endif

  SP_DMB();
#endif /* CONFIG_SMP */
}

/****************************************************************************
 * Name: spin_unlockr
 *
 * Description:
 *   Release one count on a spinlock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to unlock.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

void spin_unlockr(FAR struct spinlock_s *lock)
{
#ifdef CONFIG_SMP
  irqstate_t flags;
#ifdef CONFIG_SPINLOCK_LOCKDOWN
  uint8_t cpu = this_cpu();
#endif

  /* Disable interrupts (all CPUs) */

  flags = up_irq_save();

#ifdef CONFIG_SPINLOCK_LOCKDOWN
  /* REVISIT:  What happens if this thread took the lock on a different CPU,
   * was suspended, then reassigned to this CPU where it runs and eventually
   * calls spin_unlock(). One solution might be to lock a thread to a CPU if
   * it holds a spinlock.  That would assure that it never runs on any other
   * CPU and avoids such complexities.
   */

  DEBUGASSERT(lock != NULL && lock->sp_lock == SP_LOCKED &&
              lock->sp_cpu == this_cpu() && lock->sp_count > 0);

  /* Do we already hold the lock? */

  if (lock->sp_cpu == cpu)
#else
  /* The alternative is to allow the lock to be released from any CPU */

  DEBUGASSERT(lock != NULL && lock->sp_lock == SP_LOCKED &&
              lock->sp_count > 0);
#endif

    {
      /* Yes... just decrement the number of references we have on the lock */

      if (lock->sp_count <= 1)
        {
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
          /* Notify that we are unlocking the spinlock */

          sched_note_spinunlock(this_task(), &lock->sp_lock);
#endif
          /* The count must decremented to zero */

          lock->sp_count = 0;
          lock->sp_cpu   = IMPOSSIBLE_CPU;
          lock->sp_lock  = SP_UNLOCKED;
        }
      else
        {
          lock->sp_count--;
        }
    }

  up_irq_restore(flags);

#else /* CONFIG_SMP */

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  /* Notify that we are unlocking the spinlock */

  sched_note_spinunlock(this_task(), &lock->sp_lock);
#endif

  /* Just mark the spinlock unlocked */

  DEBUGASSERT(lock != NULL && lock->sp_lock == SP_LOCKED);
  lock->sp_lock  = SP_UNLOCKED;

#endif /* CONFIG_SMP */
}

/****************************************************************************
 * Name: spin_setbit
 *
 * Description:
 *   Makes setting a CPU bit in a bitset an atomic action
 *
 * Input Parameters:
 *   set     - A reference to the bitset to set the CPU bit in
 *   cpu     - The bit number to be set
 *   setlock - A reference to the lock protecting the set
 *   orlock  - Will be set to SP_LOCKED while holding setlock
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spin_setbit(FAR volatile cpu_set_t *set, unsigned int cpu,
                 FAR volatile spinlock_t *setlock,
                 FAR volatile spinlock_t *orlock)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  cpu_set_t prev;
#endif
  irqstate_t flags;

  /* Disable local interrupts to prevent being re-entered from an interrupt
   * on the same CPU.  This may not effect interrupt behavior on other CPUs.
   */

  flags = up_irq_save();

  /* Then, get the 'setlock' spinlock */

  spin_lock(setlock);

  /* Then set the bit and mark the 'orlock' as locked */

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  prev    = *set;
#endif
  *set   |= (1 << cpu);
  *orlock = SP_LOCKED;

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  if (prev == 0)
    {
      /* Notify that we have locked the spinlock */

      sched_note_spinlocked(this_task(), orlock);
    }
#endif

  /* Release the 'setlock' and restore local interrupts */

  spin_unlock(setlock);
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: spin_clrbit
 *
 * Description:
 *   Makes clearing a CPU bit in a bitset an atomic action
 *
 * Input Parameters:
 *   set     - A reference to the bitset to set the CPU bit in
 *   cpu     - The bit number to be set
 *   setlock - A reference to the lock protecting the set
 *   orlock  - Will be set to SP_UNLOCKED if all bits become cleared in set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spin_clrbit(FAR volatile cpu_set_t *set, unsigned int cpu,
                 FAR volatile spinlock_t *setlock,
                 FAR volatile spinlock_t *orlock)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  cpu_set_t prev;
#endif
  irqstate_t flags;

  /* Disable local interrupts to prevent being re-entered from an interrupt
   * on the same CPU.  This may not effect interrupt behavior on other CPUs.
   */

  flags = up_irq_save();

  /* First, get the 'setlock' spinlock */

  spin_lock(setlock);

  /* Then clear the bit in the CPU set.  Set/clear the 'orlock' depending
   * upon the resulting state of the CPU set.
   */

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  prev    = *set;
#endif
  *set   &= ~(1 << cpu);
  *orlock = (*set != 0) ? SP_LOCKED : SP_UNLOCKED;

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  if (prev != 0 && *set == 0)
    {
      /* Notify that we have unlocked the spinlock */

      sched_note_spinunlock(this_task(), orlock);
    }
#endif

  /* Release the 'setlock' and restore local interrupts */

  spin_unlock(setlock);
  up_irq_restore(flags);
}

#endif /* CONFIG_SPINLOCK */

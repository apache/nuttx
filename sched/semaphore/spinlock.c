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

#include <sched.h>
#include <assert.h>

#include <nuttx/spinlock.h>

#include "sched/sched.h"

#ifdef CONFIG_SPINLOCK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMPOSSIBLE_CPU 0xff

/* REVISIT:  What happens if a thread taks a spinlock while running on one
 * CPU, but is suspended, then reassigned to another CPU where it runs and
 * eventually calls spinunlock().  One solution might be to lock a thread to
 * a CPU if it holds a spinlock.  That would assure that it never runs on
 * any other CPU and avoids such complexities.
 */

#undef CONFIG_SPINLOCK_LOCKDOWN /* Feature not yet available */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spinlock_initialize
 *
 * Description:
 *   Initialize a spinlock object to its initial, unlocked state.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to be initialized.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void spinlock_initialize(FAR struct spinlock_s *lock)
{
  DEBUGASSERT(lock != NULL);

  lock->sp_lock  = SP_UNLOCKED;
#ifdef CONFIG_SMP
  lock->sp_cpu   = IMPOSSIBLE_CPU;
  lock->sp_count = 0;
#endif
}

/****************************************************************************
 * Name: spinlock
 *
 * Description:
 *   If this CPU does not already hold the spinlock, then loop until the
 *   spinlock is successfully locked.
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

void spinlock(FAR struct spinlock_s *lock)
{
#ifdef CONFIG_SMP
  irqstate_t flags;
  uint8_t cpu = this_cpu();

  /* Disable interrupts (all CPUs) */

  flags = irqsave();

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
       * to another CPU where it runs and eventually calls spinunlock().
       * One solution might be to lock a thread to a CPU if it holds a
       * spinlock.  That would assure that it never runs on any other CPU
       * and avoids such complexities.
       */

#  warning Missing logic
#endif
      /* Take the lock.  REVISIT:  We should set an indication in the TCB
       * that the thread is spinning.  This might be useful in determining
       * some scheduling actions?
       */

      while (up_testset(&lock->sp_lock) == SP_LOCKED)
        {
          irqrestore(flags);
          sched_yield();
          flags = irqsave();
        }

      /* Take one count on the lock */

      lock->sp_cpu   = cpu;
      lock->sp_count = 1;
    }

  irqrestore(flags);

#else /* CONFIG_SMP */

  /* Take the lock.  REVISIT:  We should set an indication in the TCB that
   * the thread is spinning.  This might be useful in determining some
   * scheduling actions?
   */

  while (up_testset(&lock->sp_lock) == SP_LOCKED)
    {
      sched_yield();
    }

#endif /* CONFIG_SMP */
}

/****************************************************************************
 * Name: spinunlock
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

void spinunlock(FAR struct spinlock_s *lock)
{
#ifdef CONFIG_SMP
  irqstate_t flags;
#ifdef CONFIG_SPINLOCK_LOCKDOWN
  uint8_t cpu = this_cpu();
#endif

  /* Disable interrupts (all CPUs) */

  flags = irqsave();

#ifdef CONFIG_SPINLOCK_LOCKDOWN
  /* REVISIT:  What happens if this thread took the lock on a different CPU,
   * was suspended, then reassigned to this CPU where it runs and eventually
   * calls spinunlock(). One solution might be to lock a thread to a CPU if
   * it holds a spinlock.  That would assure that it never runs on any other
   * CPU and avoids such complexities.
   */

  DEBUGASSERT(lock != NULL && lock->sp-lock = SP_LOCKED &&
              lock->sp_cpu == this_cpu() && lock->sp_count > 0);

  /* Do we already hold the lock? */

  if (lock->sp_cpu == cpu)
#else
  DEBUGASSERT(lock != NULL && lock->sp-lock = SP_LOCKED &&
              lock->sp_count > 0);
#endif

    {
      /* Yes... just decrement the number of references we have on the lock */

      if (lock->sp_count <= 1)
        {
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

  irqrestore(flags);

#else /* CONFIG_SMP */
  /* Just mark the spinlock unlocked */

  DEBUGASSERT(lock != NULL && lock->sp-lock = SP_LOCKED);
  lock->sp_lock  = SP_UNLOCKED; 

#endif /* CONFIG_SMP */
}

#endif /* CONFIG_SPINLOCK */

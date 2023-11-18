/****************************************************************************
 * sched/semaphore/spinlock.c
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
#include <stdatomic.h>
#include <sched.h>
#include <assert.h>

#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>
#include <arch/irq.h>

#if defined(CONFIG_TICKET_SPINLOCK) || defined(CONFIG_RW_SPINLOCK)
#  include <stdatomic.h>
#endif

#include "sched/sched.h"

#if defined(CONFIG_SPINLOCK) || defined(CONFIG_TICKET_SPINLOCK)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spin_lock
 *
 * Description:
 *   If this CPU does not already hold the spinlock, then loop until the
 *   spinlock is successfully locked.
 *
 *   This implementation is non-reentrant and is prone to deadlocks in
 *   the case that any logic on the same CPU attempts to take the lock
 *   more than once.
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

  sched_note_spinlock(this_task(), lock, NOTE_SPINLOCK_LOCK);
#endif

#ifdef CONFIG_TICKET_SPINLOCK
  unsigned short ticket =
    atomic_fetch_add((FAR atomic_ushort *)&lock->tickets.next, 1);
  while (atomic_load((FAR atomic_ushort *)&lock->tickets.owner) != ticket)
#else /* CONFIG_SPINLOCK */
  while (up_testset(lock) == SP_LOCKED)
#endif
    {
      SP_DSB();
      SP_WFE();
    }

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  /* Notify that we have the spinlock */

  sched_note_spinlock(this_task(), lock, NOTE_SPINLOCK_LOCKED);
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
#ifdef CONFIG_TICKET_SPINLOCK
  unsigned short ticket =
    atomic_fetch_add((FAR atomic_ushort *)&lock->tickets.next, 1);
  while (atomic_load((FAR atomic_ushort *)&lock->tickets.owner) != ticket)
#else /* CONFIG_TICKET_SPINLOCK */
  while (up_testset(lock) == SP_LOCKED)
#endif
    {
      SP_DSB();
      SP_WFE();
    }

  SP_DMB();
}

/****************************************************************************
 * Name: spin_trylock
 *
 * Description:
 *   Try once to lock the spinlock.  Do not wait if the spinlock is already
 *   locked.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   false   - Failure, the spinlock was already locked
 *   true    - Success, the spinlock was successfully locked
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

bool spin_trylock(FAR volatile spinlock_t *lock)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  /* Notify that we are waiting for a spinlock */

  sched_note_spinlock(this_task(), lock, NOTE_SPINLOCK_LOCK);
#endif

#ifdef CONFIG_TICKET_SPINLOCK
  unsigned short ticket =
    atomic_load((FAR atomic_ushort *)&lock->tickets.next);

  spinlock_t old =
    {
        {
          ticket, ticket
        }
    };

  spinlock_t new =
    {
        {
          ticket, ticket + 1
        }
    };

  if (!atomic_compare_exchange_strong((FAR atomic_uint *)&lock->value,
                                      &old.value, new.value))
#else /* CONFIG_TICKET_SPINLOCK */
  if (up_testset(lock) == SP_LOCKED)
#endif /* CONFIG_TICKET_SPINLOCK */
    {
#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
      /* Notify that we abort for a spinlock */

      sched_note_spinlock(this_task(), lock, NOTE_SPINLOCK_ABORT);
#endif
      SP_DSB();
      return false;
    }

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
  /* Notify that we have the spinlock */

  sched_note_spinlock(this_task(), lock, NOTE_SPINLOCK_LOCKED);
#endif
  SP_DMB();
  return true;
}

/****************************************************************************
 * Name: spin_trylock_wo_note
 *
 * Description:
 *   Try once to lock the spinlock.  Do not wait if the spinlock is already
 *   locked.
 *
 *   This implementation is the same as the above spin_trylock() except that
 *   it does not perform instrumentation logic.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   false   - Failure, the spinlock was already locked
 *   true    - Success, the spinlock was successfully locked
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

bool spin_trylock_wo_note(FAR volatile spinlock_t *lock)
{
#ifdef CONFIG_TICKET_SPINLOCK
  unsigned short ticket =
    atomic_load((FAR atomic_ushort *)&lock->tickets.next);

  spinlock_t old =
    {
        {
          ticket, ticket
        }
    };

  spinlock_t new =
    {
        {
          ticket, ticket + 1
        }
    };

  if (!atomic_compare_exchange_strong((FAR atomic_uint *)&lock->value,
                                      &old.value, new.value))
#else /* CONFIG_TICKET_SPINLOCK */
  if (up_testset(lock) == SP_LOCKED)
#endif /* CONFIG_TICKET_SPINLOCK */
    {
      SP_DSB();
      return false;
    }

  SP_DMB();
  return true;
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

  sched_note_spinlock(this_task(), lock, NOTE_SPINLOCK_UNLOCK);
#endif

  SP_DMB();
#ifdef CONFIG_TICKET_SPINLOCK
  atomic_fetch_add((FAR atomic_ushort *)&lock->tickets.owner, 1);
#else
  *lock = SP_UNLOCKED;
#endif
  SP_DSB();
  SP_SEV();
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
#ifdef CONFIG_TICKET_SPINLOCK
  atomic_fetch_add((FAR atomic_ushort *)&lock->tickets.owner, 1);
#else
  *lock = SP_UNLOCKED;
#endif
  SP_DSB();
  SP_SEV();
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

#ifdef CONFIG_SMP
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

      sched_note_spinlock(this_task(), orlock, NOTE_SPINLOCK_LOCKED);
    }
#endif

  /* Release the 'setlock' and restore local interrupts */

  spin_unlock(setlock);
  up_irq_restore(flags);
}
#endif

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

#ifdef CONFIG_SMP
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

      sched_note_spinlock(this_task(), orlock, NOTE_SPINLOCK_UNLOCK);
    }
#endif

  /* Release the 'setlock' and restore local interrupts */

  spin_unlock(setlock);
  up_irq_restore(flags);
}
#endif

#ifdef CONFIG_RW_SPINLOCK

/****************************************************************************
 * Name: read_lock
 *
 * Description:
 *   If this task does not already hold the spinlock, then loop until the
 *   spinlock is successfully locked.
 *
 *   This implementation is non-reentrant and set a bit of lock.
 *
 *  The priority of reader is higher than writter if a reader hold the
 *  lock, a new reader can get its lock but writer can't get this lock.
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

void read_lock(FAR volatile rwlock_t *lock)
{
  while (true)
    {
      int old = atomic_load((FAR atomic_int *)lock);
      if (old <= RW_SP_WRITE_LOCKED)
        {
          DEBUGASSERT(old == RW_SP_WRITE_LOCKED);
          SP_DSB();
          SP_WFE();
        }
      else if(atomic_compare_exchange_strong((FAR atomic_int *)lock,
                                             &old, old + 1))
        {
          break;
        }
    }

  SP_DMB();
}

/****************************************************************************
 * Name: read_trylock
 *
 * Description:
 *   If this task does not already hold the spinlock, then try to get the
 * lock.
 *
 *   This implementation is non-reentrant and set a bit of lock.
 *
 *  The priority of reader is higher than writter if a reader hold the
 *  lock, a new reader can get its lock but writer can't get this lock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   false   - Failure, the spinlock was already locked
 *   true    - Success, the spinlock was successfully locked
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

bool read_trylock(FAR volatile rwlock_t *lock)
{
  while (true)
    {
      int old = atomic_load((FAR atomic_int *)lock);
      if (old <= RW_SP_WRITE_LOCKED)
        {
          DEBUGASSERT(old == RW_SP_WRITE_LOCKED);
          return false;
        }
      else if (atomic_compare_exchange_strong((FAR atomic_int *)lock,
                                              &old, old + 1))
        {
          break;
        }
    }

  SP_DMB();
  return true;
}

/****************************************************************************
 * Name: read_unlock
 *
 * Description:
 *   Release a bit on a non-reentrant spinlock.
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

void read_unlock(FAR volatile rwlock_t *lock)
{
  DEBUGASSERT(atomic_load((FAR atomic_int *)lock) >= RW_SP_READ_LOCKED);

  SP_DMB();
  atomic_fetch_sub((FAR atomic_int *)lock, 1);
  SP_DSB();
  SP_SEV();
}

/****************************************************************************
 * Name: write_lock
 *
 * Description:
 *   If this task does not already hold the spinlock, then loop until the
 *   spinlock is successfully locked.
 *
 *   This implementation is non-reentrant and set all bit on lock to avoid
 *   readers and writers.
 *
 *  The priority of reader is higher than writter if a reader hold the
 *  lock, a new reader can get its lock but writer can't get this lock.
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

void write_lock(FAR volatile rwlock_t *lock)
{
  int zero = RW_SP_UNLOCKED;

  while (!atomic_compare_exchange_strong((FAR atomic_int *)lock,
                                         &zero, RW_SP_WRITE_LOCKED))
    {
      SP_DSB();
      SP_WFE();
    }

  SP_DMB();
}

/****************************************************************************
 * Name: write_trylock
 *
 * Description:
 *   If this task does not already hold the spinlock, then loop until the
 *   spinlock is successfully locked.
 *
 *   This implementation is non-reentrant and set all bit on lock to avoid
 *   readers and writers.
 *
 *  The priority of reader is higher than writter if a reader hold the
 *  lock, a new reader can get its lock but writer can't get this lock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to lock.
 *
 * Returned Value:
 *   false   - Failure, the spinlock was already locked
 *   true    - Success, the spinlock was successfully locked
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

bool write_trylock(FAR volatile rwlock_t *lock)
{
  int zero = RW_SP_UNLOCKED;

  if (atomic_compare_exchange_strong((FAR atomic_int *)lock,
                                     &zero, RW_SP_WRITE_LOCKED))
    {
      SP_DMB();
      return true;
    }

  SP_DSB();
  return false;
}

/****************************************************************************
 * Name: write_unlock
 *
 * Description:
 *   Release write lock on a non-reentrant spinlock.
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

void write_unlock(FAR volatile rwlock_t *lock)
{
  /* Ensure this cpu already get write lock */

  DEBUGASSERT(atomic_load((FAR atomic_int *)lock) == RW_SP_WRITE_LOCKED);

  SP_DMB();
  atomic_store((FAR atomic_int *)lock, RW_SP_UNLOCKED);
  SP_DSB();
  SP_SEV();
}

#endif /* CONFIG_RW_SPINLOCK */
#endif /* CONFIG_SPINLOCK */

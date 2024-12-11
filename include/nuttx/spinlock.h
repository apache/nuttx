/****************************************************************************
 * include/nuttx/spinlock.h
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

#ifndef __INCLUDE_NUTTX_SPINLOCK_H
#define __INCLUDE_NUTTX_SPINLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <stdint.h>

#include <nuttx/compiler.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#if defined(CONFIG_TICKET_SPINLOCK) || defined(CONFIG_RW_SPINLOCK)
#  include <nuttx/atomic.h>
#endif

#include <nuttx/spinlock_type.h>

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory barriers may be provided in arch/spinlock.h
 *
 *   DMB - Data memory barrier.  Assures writes are completed to memory.
 *   DSB - Data synchronization barrier.
 */

#undef __SP_UNLOCK_FUNCTION
#if !defined(UP_DMB)
#  define UP_DMB()
#else
#  define __SP_UNLOCK_FUNCTION 1
#endif

#if !defined(UP_DSB)
#  define UP_DSB()
#endif

#if !defined(UP_WFE)
#  define UP_WFE()
#endif

#if !defined(UP_SEV)
#  define UP_SEV()
#endif

#if !defined(__SP_UNLOCK_FUNCTION) && (defined(CONFIG_TICKET_SPINLOCK) || \
     defined(CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS))
#  define __SP_UNLOCK_FUNCTION 1
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS
void sched_note_spinlock_lock(FAR volatile spinlock_t *spinlock);
void sched_note_spinlock_locked(FAR volatile spinlock_t *spinlock);
void sched_note_spinlock_abort(FAR volatile spinlock_t *spinlock);
void sched_note_spinlock_unlock(FAR volatile spinlock_t *spinlock);
#else
#  define sched_note_spinlock_lock(spinlock)
#  define sched_note_spinlock_locked(spinlock)
#  define sched_note_spinlock_abort(spinlock)
#  define sched_note_spinlock_unlock(spinlock)
#endif

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

/****************************************************************************
 * Name: up_testset
 *
 * Description:
 *   Perform an atomic test and set operation on the provided spinlock.
 *
 *   This function must be provided via the architecture-specific logic.
 *
 * Input Parameters:
 *   lock  - A reference to the spinlock object.
 *
 * Returned Value:
 *   The spinlock is always locked upon return.  The previous value of the
 *   spinlock variable is returned, either SP_LOCKED if the spinlock was
 *   previously locked (meaning that the test-and-set operation failed to
 *   obtain the lock) or SP_UNLOCKED if the spinlock was previously unlocked
 *   (meaning that we successfully obtained the lock).
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_HAVE_TESTSET)
spinlock_t up_testset(FAR volatile spinlock_t *lock);
#else
static inline spinlock_t up_testset(FAR volatile spinlock_t *lock)
{
  irqstate_t flags;
  spinlock_t ret;

  flags = up_irq_save();

  ret = *lock;

  if (ret == SP_UNLOCKED)
    {
      *lock = SP_LOCKED;
    }

  up_irq_restore(flags);

  return ret;
}
#endif

/****************************************************************************
 * Name: spin_lock_init
 *
 * Description:
 *   Initialize a non-reentrant spinlock object to its initial,
 *   unlocked state.
 *
 * Input Parameters:
 *   lock  - A reference to the spinlock object to be initialized.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

/* void spin_lock_init(FAR spinlock_t *lock); */
#define spin_lock_init(l) do { *(l) = SP_UNLOCKED; } while (0)

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

#ifdef CONFIG_SPINLOCK
static inline_function void spin_lock_wo_note(FAR volatile spinlock_t *lock)
{
#ifdef CONFIG_TICKET_SPINLOCK
  int ticket = atomic_fetch_add(&lock->next, 1);
  while (atomic_read(&lock->owner) != ticket)
#else /* CONFIG_TICKET_SPINLOCK */
  while (up_testset(lock) == SP_LOCKED)
#endif
    {
      UP_DSB();
      UP_WFE();
    }

  UP_DMB();
}
#endif /* CONFIG_SPINLOCK */

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

#ifdef CONFIG_SPINLOCK
static inline_function void spin_lock(FAR volatile spinlock_t *lock)
{
  /* Notify that we are waiting for a spinlock */

  sched_note_spinlock_lock(lock);

  /* Lock without trace note */

  spin_lock_wo_note(lock);

  /* Notify that we have the spinlock */

  sched_note_spinlock_locked(lock);
}
#endif /* CONFIG_SPINLOCK */

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
 *   SP_LOCKED   - Failure, the spinlock was already locked
 *   SP_UNLOCKED - Success, the spinlock was successfully locked
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function bool
spin_trylock_wo_note(FAR volatile spinlock_t *lock)
{
#ifdef CONFIG_TICKET_SPINLOCK
  if (!atomic_cmpxchg(&lock->next, &lock->owner,
                      atomic_read(&lock->next) + 1))
#else /* CONFIG_TICKET_SPINLOCK */
  if (up_testset(lock) == SP_LOCKED)
#endif /* CONFIG_TICKET_SPINLOCK */
    {
      UP_DSB();
      return false;
    }

  UP_DMB();
  return true;
}
#endif /* CONFIG_SPINLOCK */

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
 *   SP_LOCKED   - Failure, the spinlock was already locked
 *   SP_UNLOCKED - Success, the spinlock was successfully locked
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function bool spin_trylock(FAR volatile spinlock_t *lock)
{
  bool locked;

  /* Notify that we are waiting for a spinlock */

  sched_note_spinlock_lock(lock);

  /* Try lock without trace note */

  locked = spin_trylock_wo_note(lock);
  if (locked)
    {
      /* Notify that we have the spinlock */

      sched_note_spinlock_locked(lock);
    }
  else
    {
      /* Notify that we abort for a spinlock */

      sched_note_spinlock_abort(lock);
    }

  return locked;
}
#endif /* CONFIG_SPINLOCK */

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

#ifdef CONFIG_SPINLOCK
static inline_function void
spin_unlock_wo_note(FAR volatile spinlock_t *lock)
{
  UP_DMB();
#ifdef CONFIG_TICKET_SPINLOCK
  atomic_fetch_add(&lock->owner, 1);
#else
  *lock = SP_UNLOCKED;
#endif
  UP_DSB();
  UP_SEV();
}
#endif /* CONFIG_SPINLOCK */

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

#ifdef CONFIG_SPINLOCK
#  ifdef __SP_UNLOCK_FUNCTION
static inline_function void spin_unlock(FAR volatile spinlock_t *lock)
{
  /* Unlock without trace note */

  spin_unlock_wo_note(lock);

  /* Notify that we are unlocking the spinlock */

  sched_note_spinlock_unlock(lock);
}
#  else
#    define spin_unlock(l)  do { *(l) = SP_UNLOCKED; } while (0)
#  endif
#endif /* CONFIG_SPINLOCK */

/****************************************************************************
 * Name: spin_is_locked
 *
 * Description:
 *   Release one count on a non-reentrant spinlock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to test.
 *
 * Returned Value:
 *   A boolean value: true the spinlock is locked; false if it is unlocked.
 *
 ****************************************************************************/

/* bool spin_islocked(FAR spinlock_t lock); */
#ifdef CONFIG_TICKET_SPINLOCK
#  define spin_is_locked(l) \
    (atomic_read(&(*l).owner) != atomic_read(&(*l).next))
#else
#  define spin_is_locked(l) (*(l) == SP_LOCKED)
#endif

/****************************************************************************
 * Name: spin_lock_irqsave_wo_note
 *
 * Description:
 *   This function is no trace version of spin_lock_irqsave()
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function
irqstate_t spin_lock_irqsave_wo_note(FAR volatile spinlock_t *lock)
{
  irqstate_t flags;
  flags = up_irq_save();

  spin_lock_wo_note(lock);

  return flags;
}
#else
#  define spin_lock_irqsave_wo_note(l) ((void)(l), up_irq_save())
#endif

/****************************************************************************
 * Name: spin_lock_irqsave
 *
 * Description:
 *   If SMP is enabled:
 *     Disable local interrupts and take the lock spinlock and return
 *     the interrupt state.
 *
 *     NOTE: This API is very simple to protect data (e.g. H/W register
 *     or internal data structure) in SMP mode. But do not use this API
 *     with kernel APIs which suspend a caller thread. (e.g. nxsem_wait)
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_save().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock. not NULL.
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to spin_lock_irqsave(lock);
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function
irqstate_t spin_lock_irqsave(FAR volatile spinlock_t *lock)
{
  irqstate_t flags;

  /* Notify that we are waiting for a spinlock */

  sched_note_spinlock_lock(lock);

  /* Lock without trace note */

  flags = spin_lock_irqsave_wo_note(lock);

  /* Notify that we have the spinlock */

  sched_note_spinlock_locked(lock);

  return flags;
}
#else
#  define spin_lock_irqsave(l) ((void)(l), up_irq_save())
#endif

/****************************************************************************
 * Name: spin_trylock_irqsave_wo_note
 *
 * Description:
 *   Try once to lock the spinlock.  Do not wait if the spinlock is already
 *   locked.
 *
 *   This implementation is the same as the above spin_trylock() except that
 *   it does not perform instrumentation logic.
 *
 * Input Parameters:
 *   lock  - A reference to the spinlock object to lock.
 *   flags - flag of interrupts status
 *
 * Returned Value:
 *   SP_LOCKED   - Failure, the spinlock was already locked
 *   SP_UNLOCKED - Success, the spinlock was successfully locked
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
#  define spin_trylock_irqsave_wo_note(l, f) \
({ \
  f = up_irq_save(); \
  spin_trylock_wo_note(l) ? \
  true : ({ up_irq_restore(f); false; }); \
})
#else
#  define spin_trylock_irqsave_wo_note(l, f) \
({ \
  (void)(l); \
  f = up_irq_save(); \
  true; \
})
#endif /* CONFIG_SPINLOCK */

/****************************************************************************
 * Name: spin_trylock_irqsave
 *
 * Description:
 *   Try once to lock the spinlock.  Do not wait if the spinlock is already
 *   locked.
 *
 * Input Parameters:
 *   lock  - A reference to the spinlock object to lock.
 *   flags - flag of interrupts status
 *
 * Returned Value:
 *   SP_LOCKED   - Failure, the spinlock was already locked
 *   SP_UNLOCKED - Success, the spinlock was successfully locked
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
#  define spin_trylock_irqsave(l, f) \
({ \
  f = up_irq_save(); \
  spin_trylock(l) ? \
  true : ({ up_irq_restore(f); false; }); \
})
#else
#  define spin_trylock_irqsave(l, f) \
({ \
  (void)(l); \
  f = up_irq_save(); \
  true; \
})
#endif /* CONFIG_SPINLOCK */

/****************************************************************************
 * Name: spin_unlock_irqrestore_wo_note
 *
 * Description:
 *   This function is no trace version of spin_unlock_irqrestore()
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function
void spin_unlock_irqrestore_wo_note(FAR volatile spinlock_t *lock,
                                    irqstate_t flags)
{
  spin_unlock_wo_note(lock);

  up_irq_restore(flags);
}
#else
#  define spin_unlock_irqrestore_wo_note(l, f) ((void)(l), up_irq_restore(f))
#endif

/****************************************************************************
 * Name: spin_unlock_irqrestore
 *
 * Description:
 *   If SMP is enabled:
 *     Release the lock and restore the interrupt state as it was prior
 *     to the previous call to spin_lock_irqsave(lock).
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_restore().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock. not NULL
 *
 *   flags - The architecture-specific value that represents the state of
 *           the interrupts prior to the call to spin_lock_irqsave(lock);
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function
void spin_unlock_irqrestore(FAR volatile spinlock_t *lock,
                            irqstate_t flags)
{
  /* Unlock without trace note */

  spin_unlock_irqrestore_wo_note(lock, flags);

  /* Notify that we are unlocking the spinlock */

  sched_note_spinlock_unlock(lock);
}
#else
#  define spin_unlock_irqrestore(l, f) ((void)(l), up_irq_restore(f))
#endif

#if defined(CONFIG_RW_SPINLOCK)

/****************************************************************************
 * Name: rwlock_init
 *
 * Description:
 *   Initialize a non-reentrant spinlock object to its initial,
 *   unlocked state.
 *
 * Input Parameters:
 *   lock  - A reference to the spinlock object to be initialized.
 *
 * Returned Value:
 *   None.
 *
 *
 ****************************************************************************/

#define rwlock_init(l) do { *(l) = RW_SP_UNLOCKED; } while(0)

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

static inline_function void read_lock(FAR volatile rwlock_t *lock)
{
  while (true)
    {
      int old = atomic_read(lock);
      if (old <= RW_SP_WRITE_LOCKED)
        {
          DEBUGASSERT(old == RW_SP_WRITE_LOCKED);
          UP_DSB();
          UP_WFE();
        }
      else if(atomic_cmpxchg(lock, &old, old + 1))
        {
          break;
        }
    }

  UP_DMB();
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

static inline_function bool read_trylock(FAR volatile rwlock_t *lock)
{
  while (true)
    {
      int old = atomic_read(lock);
      if (old <= RW_SP_WRITE_LOCKED)
        {
          DEBUGASSERT(old == RW_SP_WRITE_LOCKED);
          return false;
        }
      else if (atomic_cmpxchg(lock, &old, old + 1))
        {
          break;
        }
    }

  UP_DMB();
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

static inline_function void read_unlock(FAR volatile rwlock_t *lock)
{
  DEBUGASSERT(atomic_read(lock) >= RW_SP_READ_LOCKED);

  UP_DMB();
  atomic_fetch_sub(lock, 1);
  UP_DSB();
  UP_SEV();
}

/****************************************************************************
 * Name: write_lock
 *
 * Description:
 *   If this CPU does not already hold the spinlock, then loop until the
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

static inline_function void write_lock(FAR volatile rwlock_t *lock)
{
  int zero = RW_SP_UNLOCKED;

  while (!atomic_cmpxchg(lock, &zero, RW_SP_WRITE_LOCKED))
    {
      UP_DSB();
      UP_WFE();
    }

  UP_DMB();
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

static inline_function bool write_trylock(FAR volatile rwlock_t *lock)
{
  int zero = RW_SP_UNLOCKED;

  if (atomic_cmpxchg(lock, &zero, RW_SP_WRITE_LOCKED))
    {
      UP_DMB();
      return true;
    }

  UP_DSB();
  return false;
}

/****************************************************************************
 * Name: write_unlock
 *
 * Description:
 *   Release all bit on a non-reentrant spinlock.
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

static inline_function void write_unlock(FAR volatile rwlock_t *lock)
{
  /* Ensure this cpu already get write lock */

  DEBUGASSERT(atomic_read(lock) == RW_SP_WRITE_LOCKED);

  UP_DMB();
  atomic_set(lock, RW_SP_UNLOCKED);
  UP_DSB();
  UP_SEV();
}

/****************************************************************************
 * Name: read_lock_irqsave
 *
 * Description:
 *   If SMP is enabled:
 *     The argument lock should be specified,
 *     disable local interrupts and take the lock spinlock and return
 *     the interrupt state.
 *
 *     NOTE: This API is very simple to protect data (e.g. H/W register
 *     or internal data structure) in SMP mode. Do not use this API
 *     with kernel APIs which suspend a caller thread. (e.g. nxsem_wait)
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_save().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock, not NULL.
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to write_lock_irqsave(lock);
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
irqstate_t read_lock_irqsave(FAR rwlock_t *lock);
#else
#  define read_lock_irqsave(l) ((void)(l), up_irq_save())
#endif

/****************************************************************************
 * Name: read_unlock_irqrestore
 *
 * Description:
 *   If SMP is enabled:
 *     The argument lock should be specified, release the lock and
 *     restore the interrupt state as it was prior to the previous call to
 *     read_lock_irqsave(lock).
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_restore().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock, not NULL.
 *
 *   flags - The architecture-specific value that represents the state of
 *           the interrupts prior to the call to read_lock_irqsave(lock);
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
void read_unlock_irqrestore(FAR rwlock_t *lock, irqstate_t flags);
#else
#  define read_unlock_irqrestore(l, f) ((void)(l), up_irq_restore(f))
#endif

/****************************************************************************
 * Name: write_lock_irqsave
 *
 * Description:
 *   If SMP is enabled:
 *     The argument lock should be specified,
 *     disable local interrupts and take the lock spinlock and return
 *     the interrupt state.
 *
 *     NOTE: This API is very simple to protect data (e.g. H/W register
 *     or internal data structure) in SMP mode. But do not use this API
 *     with kernel APIs which suspend a caller thread. (e.g. nxsem_wait)
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_save().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock, not NULL.
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to write_lock_irqsave(lock);
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
irqstate_t write_lock_irqsave(FAR rwlock_t *lock);
#else
#  define write_lock_irqsave(l) ((void)(l), up_irq_save())
#endif

/****************************************************************************
 * Name: write_unlock_irqrestore
 *
 * Description:
 *   If SMP is enabled:
 *     The argument lock should be specified, release the lock and
 *     restore the interrupt state as it was prior to the previous call to
 *     write_lock_irqsave(lock).
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_restore().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock, not NULL.
 *
 *   flags - The architecture-specific value that represents the state of
 *           the interrupts prior to the call to write_lock_irqsave(lock);
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
void write_unlock_irqrestore(FAR rwlock_t *lock, irqstate_t flags);
#else
#  define write_unlock_irqrestore(l, f) ((void)(l), up_irq_restore(f))
#endif

#endif /* CONFIG_RW_SPINLOCK */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_SPINLOCK_H */

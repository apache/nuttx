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

#include <nuttx/atomic.h>

#include <nuttx/spinlock_type.h>

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_SPINLOCK_INLINE) && !defined(SPINLOCK_INLINE)
#  define SPINLOCK_INLINE static inline_function
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

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0
void nxsched_critmon_busywait(bool state, FAR void *caller);
#else
#  define nxsched_critmon_busywait(state, caller)
#endif

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

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
 * Name: rspin_lock_init
 *
 * Description:
 *   Initialize a recursive spinlock object to its initial,
 *   unlocked state.
 *
 * Input Parameters:
 *   lock  - A reference to the struct rspinlock_s object to be initialized.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline_function void rspin_lock_init(FAR rspinlock_t *lock)
{
  lock->val = 0;
}

/****************************************************************************
 * Name: spin_lock_notrace
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
#  ifdef CONFIG_SPINLOCK_INLINE
SPINLOCK_INLINE noinstrument_function
void spin_lock_notrace(FAR volatile spinlock_t *lock)
{
#ifdef CONFIG_TICKET_SPINLOCK
  int32_t ticket = atomic_add_relaxed(&lock->next, 1);
  while (atomic_read_acquire(&lock->owner) != ticket)
    {
      UP_WFE();
    }
#else /* CONFIG_TICKET_SPINLOCK */
  while (atomic_xchg_acquire((FAR atomic_t *)lock, SP_LOCKED) == SP_LOCKED)
    {
      UP_WFE();
    }
#endif
}
#  else
noinstrument_function
void spin_lock_notrace(FAR volatile spinlock_t *lock);
#  endif
#else
#  define spin_lock_notrace(lock)
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

  /* If CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0, count busy-waiting. */

  nxsched_critmon_busywait(true, return_address(0));

  /* Lock without trace note */

  spin_lock_notrace(lock);

  /* Get the lock, end counting busy-waiting */

  nxsched_critmon_busywait(false, return_address(0));

  /* Notify that we have the spinlock */

  sched_note_spinlock_locked(lock);
}
#else
#  define spin_lock(lock)
#endif /* CONFIG_SPINLOCK */

/****************************************************************************
 * Name: spin_trylock_notrace
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
spin_trylock_notrace(FAR volatile spinlock_t *lock)
{
#ifdef CONFIG_TICKET_SPINLOCK
  /* The expected value must live in a local.  A failed compare-exchange
   * writes the current value of the target object back through the
   * expected pointer, so passing &lock->owner here would clobber the
   * owner counter and make a lock held by another CPU appear unlocked.
   *
   * The exchange succeeds only when next == owner, which is the unlocked
   * state of a ticket lock.
   */

  uint32_t expected = atomic_read(&lock->owner);

  return atomic_cmpxchg_acquire(&lock->next, &expected, expected + 1);
#else /* CONFIG_TICKET_SPINLOCK */
  return atomic_xchg_acquire((FAR atomic_t *)lock, SP_LOCKED) != SP_LOCKED;
#endif /* CONFIG_TICKET_SPINLOCK */
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

  locked = spin_trylock_notrace(lock);
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
 * Name: spin_unlock_notrace
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
spin_unlock_notrace(FAR volatile spinlock_t *lock)
{
#ifdef CONFIG_TICKET_SPINLOCK
  atomic_set_release(&lock->owner, lock->owner + 1);
#else
  atomic_set_release((FAR atomic_t *)lock, SP_UNLOCKED);
#endif
  UP_SEV();
}
#else
#  define spin_unlock_notrace(lock)
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
static inline_function void spin_unlock(FAR volatile spinlock_t *lock)
{
  /* Unlock without trace note */

  spin_unlock_notrace(lock);

  /* Notify that we are unlocking the spinlock */

  sched_note_spinlock_unlock(lock);
}
#else
#  define spin_unlock(lock)
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
 * Name: spin_lock_irqsave_notrace
 *
 * Description:
 *   This function is no trace version of spin_lock_irqsave()
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function
irqstate_t spin_lock_irqsave_notrace(FAR volatile spinlock_t *lock)
{
  irqstate_t flags;
  flags = up_irq_save();

  spin_lock_notrace(lock);

  return flags;
}
#else
#  define spin_lock_irqsave_notrace(l) ((void)(l), up_irq_save())
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

  /* If CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0, count busy-waiting. */

  nxsched_critmon_busywait(true, return_address(0));

  /* Lock without trace note */

  flags = spin_lock_irqsave_notrace(lock);

  /* Get the lock, end counting busy-waiting */

  nxsched_critmon_busywait(false, return_address(0));

  /* Notify that we have the spinlock */

  sched_note_spinlock_locked(lock);

  return flags;
}
#else
#  define spin_lock_irqsave(l) ((void)(l), up_irq_save())
#endif

/****************************************************************************
 * Name: spin_lock_irqsave_nopreempt
 *
 * Description:
 *   If SMP is enabled:
 *     Disable local interrupts, sched_lock and take the lock spinlock and
 *     return the interrupt state.
 *
 *     NOTE: This API is very simple to protect data (e.g. H/W register
 *     or internal data structure) in SMP mode. But do not use this API
 *     with kernel APIs which suspend a caller thread. (e.g. nxsem_wait)
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_save() + sched_lock().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock. not NULL.
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to spin_lock_irqsave(lock);
 *
 ****************************************************************************/

static inline_function
irqstate_t spin_lock_irqsave_nopreempt(FAR volatile spinlock_t *lock)
{
  irqstate_t flags;
  flags = spin_lock_irqsave(lock);
  sched_lock();
  return flags;
}

/****************************************************************************
 * Name: rspin_lock_count
 *
 * Description:
 *   This function return rspinlock count.
 *
 *   Note that this is inherently racy unless the calling thread is
 *   holding the rspinlock.
 *
 * Parameters:
 *   lock - Recursive spinlock descriptor.
 *
 * Return Value:
 *   Rspinlock count.
 *
 ****************************************************************************/

static inline_function
uint16_t rspin_lock_count(FAR volatile rspinlock_t *lock)
{
  return RSPINLOCK_COUNT(atomic_read(&lock->val));
}

/****************************************************************************
 * Name: rspin_lock_irqsave/rspin_lock_irqsave_nopreempt
 *
 * Description:
 *   Nest supported spinlock, can support UINT8_MAX max depth.
 *   As we should not disable irq for long time, sched also locked.
 *   Similar feature with enter_critical_section, but isolate by instance.
 *
 *   If SPINLOCK is enabled:
 *     Will take spinlock each cpu first call.
 *
 *   If SPINLOCK is not enabled:
 *     Equivalent to up_irq_save() + sched_lock().
 *     Will only sched_lock once when first called.
 *
 * Input Parameters:
 *   lock - Caller specific rspinlock_s. not NULL.
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to spin_lock_irqsave(lock);
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
#  ifdef CONFIG_SPINLOCK_INLINE
SPINLOCK_INLINE noinstrument_function
void rspin_lock(FAR rspinlock_t *lock)
{
  uint32_t new_val;
  uint32_t old_val;
  uint32_t owner = (uint32_t)this_cpu() + 1u;

  /* Already owned this lock. */

  old_val = atomic_read(&lock->val);

  if (RSPINLOCK_OWNER(old_val) == owner)
    {
      DEBUGASSERT(RSPINLOCK_COUNT(old_val) + 1u <= UINT16_MAX);

      lock->local_val = old_val + 1u;
    }
  else
    {
      /* Try seize the ownership of the lock using CAS. */

      new_val = RSPINLOCK_VALUE(owner, 1u);

      for (; ; )
        {
          old_val = 0u;
          if (atomic_cmpxchg_acquire(&lock->val, &old_val, new_val))
            {
              break;
            }

          UP_WFE();
        }

      /* The local value shares the same memory location with the
       * global atomic value. So if the atomic operation success,
       * the local value should also change to the new_val.
       */

      compiler_assume(lock->local_val == new_val);
    }
}
#  else
noinstrument_function
void rspin_lock(FAR rspinlock_t *lock);
#  endif

static inline_function
irqstate_t rspin_lock_irqsave(FAR rspinlock_t *lock)
{
  irqstate_t flags = up_irq_save();
  rspin_lock(lock);
  return flags;
}
#else
#  define rspin_lock_irqsave(l) ((void)(l), up_irq_save())
#endif

static inline_function
irqstate_t rspin_lock_irqsave_nopreempt(FAR rspinlock_t *lock)
{
  irqstate_t flags = rspin_lock_irqsave(lock);
  sched_lock();

  return flags;
}

/****************************************************************************
 * Name: rspin_trylock/rspin_trylock_irqsave/rspin_trylock_irqsave_nopreempt
 *
 * Description:
 *   Nest supported spinlock, try once to lock the rspinlock, can support
 *   UINT16_MAX max depth.
 *   As we should not disable irq for long time, sched also locked.
 *   Similar feature with enter_critical_section, but isolate by instance.
 *
 *   If SPINLOCK is enabled:
 *     Will take spinlock each cpu first call.
 *
 *   If SPINLOCK is not enabled:
 *     Equivalent to up_irq_save() + sched_lock().
 *     Will only sched_lock once when first called.
 *
 * Input Parameters:
 *   lock - Caller specific rspinlock_s. not NULL.
 *
 * Returned Value:
 *   true  - Success, the spinlock was successfully locked
 *   false - Failure, the spinlock was already locked
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
#  ifdef CONFIG_SPINLOCK_INLINE
SPINLOCK_INLINE bool rspin_trylock(FAR rspinlock_t *lock)
{
  uint32_t old_val;
  bool         ret = false;
  uint32_t   owner = (uint32_t)this_cpu() + 1u;

  /* Already owned this lock. */

  old_val = atomic_read(&lock->val);

  if (RSPINLOCK_OWNER(old_val) == owner)
    {
      DEBUGASSERT(RSPINLOCK_COUNT(old_val) + 1u <= UINT16_MAX);

      lock->local_val = old_val + 1u;
      ret = true;
    }
  else if (old_val == 0u)
    {
      /* Try seize the ownership of the lock using CAS. */

      ret = atomic_cmpxchg_acquire(&lock->val, &old_val,
                                   RSPINLOCK_VALUE(owner, 1u));

      /* Mark locked when the lock is successfully acquired. */

      if (ret)
        {
          /* The local value shares the same memory location with the
           * global atomic value. So if the atomic operation success,
           * the local value should also change to the new_val.
           */

          compiler_assume(lock->local_val == RSPINLOCK_VALUE(owner, 1u));
        }
    }

  return ret;
}
#  else
bool rspin_trylock(FAR rspinlock_t *lock);
#  endif

#  define rspin_trylock_irqsave(l, f) \
    ({ \
      (f) = up_irq_save(); \
      rspin_trylock(l) ? \
      true : ({ up_irq_restore(f); false; }); \
    })
#else
#  define rspin_trylock_irqsave(l, f) \
    ({ \
      (void)(l); \
      (f) = up_irq_save(); \
      true; \
    })
#endif

#define rspin_trylock_irqsave_nopreempt(l, f) \
  ({ \
    rspin_trylock_irqsave(l, f) ? \
    ({ sched_lock(); true; }) : false; \
  })

/****************************************************************************
 * Name: spin_trylock_irqsave_notrace
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
#  define spin_trylock_irqsave_notrace(l, f) \
({ \
  f = up_irq_save(); \
  spin_trylock_notrace(l) ? \
  true : ({ up_irq_restore(f); false; }); \
})
#else
#  define spin_trylock_irqsave_notrace(l, f) \
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
 * Name: spin_unlock_irqrestore_notrace
 *
 * Description:
 *   This function is no trace version of spin_unlock_irqrestore()
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function
void spin_unlock_irqrestore_notrace(FAR volatile spinlock_t *lock,
                                irqstate_t flags)
{
  spin_unlock_notrace(lock);

  up_irq_restore(flags);
}
#else
#  define spin_unlock_irqrestore_notrace(l, f) ((void)(l), up_irq_restore(f))
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
void spin_unlock_irqrestore(FAR volatile spinlock_t *lock, irqstate_t flags)
{
  /* Unlock without trace note */

  spin_unlock_irqrestore_notrace(lock, flags);

  /* Notify that we are unlocking the spinlock */

  sched_note_spinlock_unlock(lock);
}
#else
#  define spin_unlock_irqrestore(l, f) ((void)(l), up_irq_restore(f))
#endif

/****************************************************************************
 * Name: spin_unlock_irqrestore_nopreempt
 *
 * Description:
 *   If SMP is enabled:
 *     Release the lock and restore the interrupt state, sched_unlock
 *     as it was prior to the previous call to
 *     spin_unlock_irqrestore_nopreempt(lock).
 *
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_restore() + sched_unlock().
 *
 * Input Parameters:
 *   lock - Caller specific spinlock. not NULL
 *
 *   flags - The architecture-specific value that represents the state of
 *           the interrupts prior to the call to
 *           spin_unlock_irqrestore_nopreempt(lock);
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline_function
void spin_unlock_irqrestore_nopreempt(FAR volatile spinlock_t *lock,
                                      irqstate_t flags)
{
  spin_unlock_irqrestore(lock, flags);
  sched_unlock();
}

/****************************************************************************
 * Name: rspin_unlock_irqrestore/rspin_unlock_irqrestore_nopreempt
 *
 * Description:
 *   Nest supported spinunlock, can support UINT8_MAX max depth.
 *   Should work with rspin_lock_irqsave_nopreempt().
 *   Similar feature with leave_critical_section, but isolate by instance.
 *
 *   If SPINLOCK is enabled:
 *     Will release spinlock each cpu last call.
 *
 *   If SPINLOCK is not enabled:
 *     Equivalent to sched_unlock() + up_irq_restore().
 *     Will only sched_unlock once when last called.
 *
 * Input Parameters:
 *   lock - Caller specific rspinlock_s.
 *
 *   flags - The architecture-specific value that represents the state of
 *           the interrupts prior to the call to
 *           spin_unlock_irqrestore_nopreempt(lock);
 *
 * Returned Value:
 *   true  - Indicates exiting the spinlock.
 *
 ****************************************************************************/

#ifdef CONFIG_SPINLOCK
static inline_function
bool rspin_unlock(FAR rspinlock_t *lock)
{
  bool     ret = false;
  uint32_t val = lock->local_val;

  DEBUGASSERT(RSPINLOCK_OWNER(val) == (uint32_t)this_cpu() + 1u);
  DEBUGASSERT(RSPINLOCK_COUNT(val) >= 1u);

  if (RSPINLOCK_COUNT(val) == 1u)
    {
      atomic_set_release(&lock->val, 0u);
      UP_SEV();
      ret = true;
    }
  else
    {
      lock->local_val = val - 1u;
    }

  return ret;
}

static inline_function
void rspin_unlock_irqrestore(FAR rspinlock_t *lock, irqstate_t flags)
{
  if (rspin_unlock(lock))
    {
      up_irq_restore(flags);
    }

  /* If not last rspinlock restore,  up_irq_restore should not required */
}
#else
#  define rspin_unlock_irqrestore(l, f) ((void)(l), up_irq_restore(f))
#endif

static inline_function
void rspin_unlock_irqrestore_nopreempt(FAR rspinlock_t *lock,
                                       irqstate_t flags)
{
  rspin_unlock_irqrestore(lock, flags);
  sched_unlock();
}

#ifdef CONFIG_SPINLOCK
static inline_function
uint16_t rspin_breaklock(FAR rspinlock_t *lock)
{
  uint16_t oldcount = 0u;
  uint32_t val      = lock->local_val;
  uint32_t owner    = (uint32_t)this_cpu() + 1u;

  DEBUGASSERT(owner <= UINT16_MAX);

  if (RSPINLOCK_OWNER(val) == owner)
    {
      oldcount        = RSPINLOCK_COUNT(val);
      lock->local_val = RSPINLOCK_VALUE(owner, 1u);
      rspin_unlock(lock);
    }

  return oldcount;
}

static inline_function
void rspin_restorelock(FAR rspinlock_t *lock, uint16_t count)
{
  uint32_t val;
  if (count != 0u)
    {
      rspin_lock(lock);
      val = lock->local_val;
      lock->local_val = RSPINLOCK_VALUE(RSPINLOCK_OWNER(val), count);
    }
}
#else
#  define rspin_breaklock(lock) (0)
#  define rspin_restorelock(lock, count)
#endif

#ifdef CONFIG_RW_SPINLOCK

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
 *  The reader's priority is higher than the writer's priority.  If a reader
 *  holds the lock, a new reader can get its lock but a writer can't get this
 *  lock.
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

#  ifdef CONFIG_SPINLOCK_INLINE
SPINLOCK_INLINE void read_lock(FAR volatile rwlock_t *lock)
{
  nxsched_critmon_busywait(true, return_address(0));
  for (; ; )
    {
      int32_t old = atomic_read(lock);

      if (old > RW_SP_WRITE_LOCKED)
        {
          if (atomic_cmpxchg_acquire(lock, &old, old + 1))
            {
              break;
            }

          if (old == RW_SP_WRITE_LOCKED)
            {
              UP_WFE();
            }
        }
    }

  nxsched_critmon_busywait(false, return_address(0));
}
#  else
void read_lock(FAR volatile rwlock_t *lock);
#  endif

/****************************************************************************
 * Name: read_trylock
 *
 * Description:
 *   If this task does not already hold the spinlock, then try to get the
 * lock.
 *
 *   This implementation is non-reentrant and set a bit of lock.
 *
 *  The reader's priority is higher than the writer's priority.  If a reader
 *  holds the lock, a new reader can get its lock but a writer can't get this
 *  lock.
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

#  ifdef CONFIG_SPINLOCK_INLINE
SPINLOCK_INLINE bool read_trylock(FAR volatile rwlock_t *lock)
{
  for (; ; )
    {
      int32_t old = atomic_read(lock);

      if (old <= RW_SP_WRITE_LOCKED)
        {
          DEBUGASSERT(old == RW_SP_WRITE_LOCKED);
          return false;
        }
      else if (atomic_cmpxchg_acquire(lock, &old, old + 1))
        {
          break;
        }
    }

  UP_DMB();
  return true;
}
#  else
bool read_trylock(FAR volatile rwlock_t *lock);
#  endif

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

  atomic_sub_release(lock, 1);
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
 *  The reader's priority is higher than the writer's priority.  If a reader
 *  holds the lock, a new reader can get its lock but a writer can't get this
 *  lock.
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

#  ifdef CONFIG_SPINLOCK_INLINE
SPINLOCK_INLINE void write_lock(FAR volatile rwlock_t *lock)
{
  nxsched_critmon_busywait(true, return_address(0));

  for (; ; )
    {
      int zero = RW_SP_UNLOCKED;
      if (atomic_cmpxchg_acquire((FAR atomic_int *)lock, &zero,
                                 RW_SP_WRITE_LOCKED))
        {
          break;
        }

      UP_WFE();
    }

  nxsched_critmon_busywait(false, return_address(0));
}
#  else
void write_lock(FAR volatile rwlock_t *lock);
#  endif

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
 *  The reader's priority is higher than the writer's priority.  If a reader
 *  holds the lock, a new reader can get its lock but a writer can't get this
 *  lock.
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

  return atomic_cmpxchg_acquire(lock, &zero, RW_SP_WRITE_LOCKED);
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

  atomic_set_release(lock, RW_SP_UNLOCKED);
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
static inline_function irqstate_t read_lock_irqsave(FAR rwlock_t *lock)
{
  irqstate_t ret;
  ret = up_irq_save();

  read_lock(lock);

  return ret;
}
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
static inline_function
void read_unlock_irqrestore(rwlock_t *lock, irqstate_t flags)
{
  read_unlock(lock);
  up_irq_restore(flags);
}
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
static inline_function irqstate_t write_lock_irqsave(rwlock_t *lock)
{
  irqstate_t ret;
  ret = up_irq_save();

  write_lock(lock);

  return ret;
}
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
static inline_function
void write_unlock_irqrestore(rwlock_t *lock, irqstate_t flags)
{
  write_unlock(lock);

  up_irq_restore(flags);
}
#else
#  define write_unlock_irqrestore(l, f) ((void)(l), up_irq_restore(f))
#endif

#endif /* CONFIG_RW_SPINLOCK */

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_SPINLOCK_H */

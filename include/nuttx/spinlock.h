/****************************************************************************
 * include/nuttx/spinlock.h
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
#include <stdint.h>

#ifndef CONFIG_SPINLOCK
typedef struct
{
} spinlock_t;
#else

/* The architecture specific spinlock.h header file must also provide the
 * following:
 *
 *   SP_LOCKED   - A definition of the locked state value (usually 1)
 *   SP_UNLOCKED - A definition of the unlocked state value (usually 0)
 *   spinlock_t  - The type of a spinlock memory object.
 *
 * SP_LOCKED and SP_UNLOCKED must constants of type spinlock_t.
 */

#include <arch/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory barriers may be provided in arch/spinlock.h
 *
 *   DMB - Data memory barrier.  Assures writes are completed to memory.
 *   DSB - Data synchronization barrier.
 */

#undef __SP_UNLOCK_FUNCTION
#if !defined(SP_DMB)
#  define SP_DMB()
#else
#  define __SP_UNLOCK_FUNCTION 1
#endif

#if !defined(SP_DSB)
#  define SP_DSB()
#endif

#if !defined(SP_WFE)
#  define SP_WFE()
#endif

#if !defined(SP_SEV)
#  define SP_SEV()
#endif

#if defined(CONFIG_SCHED_INSTRUMENTATION_SPINLOCKS) && !defined(__SP_UNLOCK_FUNCTION)
#  define __SP_UNLOCK_FUNCTION 1
#endif

/* If the target CPU supports a data cache then it may be necessary to
 * manage spinlocks in a special way, perhaps linking them all into a
 * special non-cacheable memory region.
 *
 *   SP_SECTION - Special storage attributes may be required to force
 *      spinlocks into a special, non-cacheable section.
 */

#if !defined(SP_SECTION)
#  define SP_SECTION
#endif

/****************************************************************************
 * Public Function Prototypes
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
 *   lock - The address of spinlock object.
 *
 * Returned Value:
 *   The spinlock is always locked upon return.  The value of previous value
 *   of the spinlock variable is returned, either SP_LOCKED if the spinlock
 *   as previously locked (meaning that the test-and-set operation failed to
 *   obtain the lock) or SP_UNLOCKED if the spinlock was previously unlocked
 *   (meaning that we successfully obtained the lock)
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_HAVE_TESTSET)
spinlock_t up_testset(volatile FAR spinlock_t *lock);
#elif !defined(CONFIG_SMP)
static inline spinlock_t up_testset(volatile FAR spinlock_t *lock)
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
 * Name: spin_initialize
 *
 * Description:
 *   Initialize a non-reentrant spinlock object to its initial,
 *   unlocked state.
 *
 * Input Parameters:
 *   lock  - A reference to the spinlock object to be initialized.
 *   state - Initial state of the spinlock {SP_LOCKED or SP_UNLOCKED)
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

/* void spin_initialize(FAR spinlock_t *lock, spinlock_t state); */
#define spin_initialize(l,s) do { *(l) = (s); } while (0)

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

void spin_lock(FAR volatile spinlock_t *lock);

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

void spin_lock_wo_note(FAR volatile spinlock_t *lock);

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

spinlock_t spin_trylock(FAR volatile spinlock_t *lock);

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

spinlock_t spin_trylock_wo_note(FAR volatile spinlock_t *lock);

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
void spin_unlock(FAR volatile spinlock_t *lock);
#else
#  define spin_unlock(l)  do { *(l) = SP_UNLOCKED; } while (0)
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

void spin_unlock_wo_note(FAR volatile spinlock_t *lock);

/****************************************************************************
 * Name: spin_islocked
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
#define spin_islocked(l) (*(l) == SP_LOCKED)

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
                 FAR volatile spinlock_t *orlock);
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
                 FAR volatile spinlock_t *orlock);
#endif

#endif /* CONFIG_SPINLOCK */
#endif /* __INCLUDE_NUTTX_SPINLOCK_H */

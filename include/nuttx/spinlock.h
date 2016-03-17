/****************************************************************************
 * include/nuttx/spinlock.h
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

#ifndef __INCLUDE_NUTTX_SPINLOCK_H
#define __INCLUDE_NUTTX_SPINLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#ifdef CONFIG_SPINLOCK

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
 * Public Types
 ****************************************************************************/

struct spinlock_s
{
  volatile spinlock_t sp_lock;  /* Indicates if the spinlock is locked or
                                 * not.  See the* values SP_LOCKED and
                                 * SP_UNLOCKED. */
#ifdef CONFIG_SMP
  uint8_t  sp_cpu;              /* CPU holding the lock */
  uint16_t sp_count;            /* The count of references by this CPU on
                                 * the lock */
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_testset
 *
 * Description:
 *   Perform and atomic test and set operation on the provided spinlock.
 *
 *   This function must be provided via the architecture-specific logoic.
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

spinlock_t up_testset(volatile FAR spinlock_t *lock);

/****************************************************************************
 * Name: spin_initialize
 *
 * Description:
 *   Initialize a non-reentrant spinlock object to its initial, unlocked state.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to be initialized.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

/* void spin_initialize(FAR spinlock_t *lock); */
#define spin_initialize(i) do { (l) = SPI_UNLOCKED; } while (0)

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

void spin_initializer(FAR struct spinlock_s *lock);

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

void spin_lockr(FAR struct spinlock_s *lock);

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

/* void spin_unlock(FAR spinlock_t *lock); */
#define spin_unlock(l)  do { *(l) = SP_UNLOCKED; } while (0)

/****************************************************************************
 * Name: spin_unlockr
 *
 * Description:
 *   Release one count on a re-entrant spinlock.
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

void spin_unlockr(FAR struct spinlock_s *lock);

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
 * Name: spin_islockedr
 *
 * Description:
 *   Release one count on a re-entrant spinlock.
 *
 * Input Parameters:
 *   lock - A reference to the spinlock object to test.
 *
 * Returned Value:
 *   A boolean value: true the spinlock is locked; false if it is unlocked.
 *
 ****************************************************************************/

/* bool spin_islockedr(FAR struct spinlock_s *lock); */
#define spin_islockedr(l) ((l)->sp_lock == SP_LOCKED)

/****************************************************************************
 * Name: spin_setbit
 *
 * Description:
 *   Makes setting a CPU bit in a bitset an atomic action
 *
 * Input Parameters:
 *   set     - A reference to the bitset to set the CPU bit in
 *   cpu     - The bit number to be set
 *   setlock - A reference to the lock lock protecting the set
 *   orlock  - Will be set to SP_LOCKED while holding setlock
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spin_setbit(FAR volatile cpu_set_t *set, unsigned int cpu,
                 FAR volatile spinlock_t *setlock,
                 FAR volatile spinlock_t *orlock);

/****************************************************************************
 * Name: spin_clrbit
 *
 * Description:
 *   Makes clearing a CPU bit in a bitset an atomic action
 *
 * Input Parameters:
 *   set     - A reference to the bitset to set the CPU bit in
 *   cpu     - The bit number to be set
 *   setlock - A reference to the lock lock protecting the set
 *   orlock  - Will be set to SP_UNLOCKED if all bits become cleared in set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spin_clrbit(FAR volatile cpu_set_t *set, unsigned int cpu,
                 FAR volatile spinlock_t *setlock,
                 FAR volatile spinlock_t *orlock);

#endif /* CONFIG_SPINLOCK */
#endif /* __INCLUDE_NUTTX_SPINLOCK_H */

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

void spinlock_initialize(FAR struct spinlock_s *lock);

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

void spinlock(FAR struct spinlock_s *lock);

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

void spinunlock(FAR struct spinlock_s *lock);

#endif /* CONFIG_SPINLOCK */
#endif /* __INCLUDE_NUTTX_SPINLOCK_H */

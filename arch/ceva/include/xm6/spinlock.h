/****************************************************************************
 * arch/ceva/include/xm6/spinlock.h
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

#ifndef __ARCH_CEVA_INCLUDE_XM6_SPINLOCK_H
#define __ARCH_CEVA_INCLUDE_XM6_SPINLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/xm6/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SP_SECTION __attribute__ ((section(".DSECT spinlock")))

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

static inline void up_dsb(void)
{
  /* MSS_BARRIER(0x638):
   * Bit [7] Internal Barrier Activation
   */
#define MSS_BARRIER 0x638

  uint32_t barrier = 0x80;

  __asm__ __volatile__
  (
    "out {cpm} %0.ui, (%1.ui).ui"
     : : "r"(barrier), "r"(MSS_BARRIER)
  );

  do
    {
      __asm__ __volatile__
      (
        "in {cpm} (%1.ui).ui, %0.ui\n"
        "nop #0x04\nnop #0x02"
        : "=r"(barrier)
        : "r"(MSS_BARRIER)
      );

      /* Wait unitl the barrier operation complete */
    }
  while ((barrier & 0x80) != 0);
#undef MSS_BARRIER
}

static inline void up_dmb(void)
{
  up_dsb(); /* use dsb instead since dmb doesn't exist on xm6 */
}

/****************************************************************************
 * Name: up_testset
 *
 * Description:
 *   Perform an atomic test and set operation on the provided spinlock.
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

static inline spinlock_t up_testset(volatile spinlock_t *lock)
{
  irqstate_t flags;
  spinlock_t old;

  /* Disable the interrupt */

  flags = up_irq_save();

  while (1)
    {
      uint32_t modc = 0;

      /* Issue exclusive read */

      __asm__ __volatile__
      (
        "nop\n"
        "LS0.ld (%1.ui).ui, %0.ui || monitor {on}\n"
        "nop #0x02"
        : "=r"(old)
        : "r"(lock)
      );

      /* Is it already locked by other? */

      if (old == SP_LOCKED)
        {
          break; /* Yes, exit */
        }

      /* Not yet, issue exclusive write */

      __asm__ __volatile__
      (
        "LS1.st %2.ui, (%1.ui).ui || monitor {off}\n"
        "mov modc.ui, %0.ui\n"
        "nop"
        : "=r"(modc)
        : "r"(lock), "r"(SP_LOCKED)
        : "memory"
      );

      /* Exclusive write success? */

      if ((modc & 0x01) == 0) /* Bit[0] Monitor status */
        {
          break; /* Yes, we are done */
        }

      /* Fail, let's try again */
    }

  /* Restore the interrupt */

  up_irq_restore(flags);

  return old;
}

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_CEVA_INCLUDE_XM6_SPINLOCK_H */

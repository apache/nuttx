/****************************************************************************
 * arch/xtensa/src/common/xtensa_testset.c
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

#include <nuttx/spinlock.h>

#include "xtensa.h"

#ifdef CONFIG_SPINLOCK

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_compareset
 *
 * Description:
 *   Wrapper for the Xtensa compare-and-set instruction. This function will
 *   atomically compare *addr to compare, and if it's the same, will set
 *   *addr to set. It will return the old value of *addr.
 *
 *   Warning: From the ISA docs: in some (unspecified) cases, the s32c1i
 *   instruction may return the *bitwise inverse* of the old mem if the
 *   mem wasn't written. This doesn't seem to happen on the ESP32, though.
 *   (Would show up directly if it did because the magic wouldn't match.)
 *
 ****************************************************************************/

static inline uint32_t xtensa_compareset(volatile uint32_t *addr,
                                         uint32_t compare,
                                         uint32_t set)
{
  __asm__ __volatile__
  (
    "WSR    %2, SCOMPARE1\n" /* Initialize SCOMPARE1 */
    "S32C1I %0, %1, 0\n"     /* Store the compare value into the lock,
                              * if the lock is the same as compare1.
                              * Otherwise, no write-access */
    : "=r"(set) : "r"(addr), "r"(compare), "0"(set)
  );

  return set;
}

/****************************************************************************
 * Public Functions
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

spinlock_t up_testset(volatile spinlock_t *lock)
{
  /* Perform the 32-bit compare and set operation */

  return xtensa_compareset((volatile uint32_t *)lock,
                           SP_UNLOCKED, SP_LOCKED);
}

#endif /* CONFIG_SPINLOCK */

/****************************************************************************
 * arch/sparc/src/common/sparc_testset.c
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

#ifdef CONFIG_SPINLOCK

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sparc_compareset
 *
 * Description:
 *   Wrapper for the Sparc compare-and-swap instruction. This function will
 *   atomically compare *addr to compare, and if it's the same, will swap
 *   *addr with set. It will return the value of set which is the old value
 *   of *addr.
 *
 * Note: The ldstub and swap instructions are available in all LEON
 * processors, while casa is optional. The CASA is a SPARC-V9 Compare and
 * Swap Alternative instruction but LEON3(GR712R) and LEON4 implements the
 * SPARC V9 Compare and Swap Alternative (CASA) instruction. The CASA
 * operates as described in the SPARC-V9 manual. This instruction is
 * privileged, except when setting ASI = 0xA (user data). All multi-core
 * LEON based components from Cobham Gaisler have casa. According to BCC
 * User's Manual the GCC option -mcpu=leon3 is required to generate SPARC-V8
 * code but support for the casa instruction.
 *
 ****************************************************************************/

static inline uint32_t sparc_compareset(volatile uint32_t *addr,
                                        uint32_t compare,
                                        uint32_t set)
{
  __asm__ __volatile__
  (
    "casa [%2] 0xb, %3, %0\n" /* Atomically compare [%2] to %3, and swap
                               * [%2] with %0 if the lock is the same as
                               * compare, otherwise, no write-access.
                               */
    : "=&r" (set) : "0" (set), "r" (addr), "r" (compare) : "memory"
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
 *   Perform an atomic compare and swap operation on the provided spinlock.
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

  return sparc_compareset((volatile uint32_t *)lock,
                           SP_UNLOCKED, SP_LOCKED);
}

#endif /* CONFIG_SPINLOCK */

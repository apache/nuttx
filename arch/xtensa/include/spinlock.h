/****************************************************************************
 * arch/xtensa/include/spinlock.h
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

#ifndef __ARCH_XTENSA_INCLUDE_SPINLOCK_H
#define __ARCH_XTENSA_INCLUDE_SPINLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/types.h>

#ifndef __ASSEMBLY__

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

#if defined(CONFIG_SPINLOCK)
static inline_function _spinlock_t up_testset(volatile _spinlock_t *lock)
{
  /* Perform the 32-bit compare and set operation */

  _spinlock_t ret;

  __asm__ __volatile__
  (
    "WSR    %2, SCOMPARE1\n" /* Initialize SCOMPARE1 */
    "S32C1I %0, %1, 0\n"     /* Store the compare value into the lock,
                              * if the lock is the same as compare1.
                              * Otherwise, no write-access */
    : "=r"(ret) : "r"(lock), "r"(UP_SP_UNLOCKED), "0"(UP_SP_LOCKED)
  );

  return ret;
}
#endif

/* See prototype in nuttx/include/nuttx/spinlock.h */

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_INCLUDE_SPINLOCK_H */

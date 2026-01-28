/****************************************************************************
 * arch/arm/include/spinlock.h
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

#ifndef __ARCH_ARM_INCLUDE_SPINLOCK_H
#define __ARCH_ARM_INCLUDE_SPINLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/types.h>
#include <arch/barriers.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#ifdef CONFIG_ARM_HAVE_WFE_SEV
#  ifndef UP_WFE
#    define UP_WFE() __asm__ __volatile__ ("wfe" : : : "memory")
#  endif
#  ifndef UP_SEV
#    define UP_SEV() __asm__ __volatile__ ("sev" : : : "memory")
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

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

#if defined(CONFIG_ARCH_HAVE_TESTSET) && \
    !defined(CONFIG_ARCH_HAVE_CUSTOM_TESTSET)
static inline_function _spinlock_t up_testset(volatile _spinlock_t *lock)
{
  _spinlock_t ret = UP_SP_UNLOCKED;

  __asm__ __volatile__
  (
    "1:                    \n"
    "ldrexb   %0, [%2]     \n"
    "cmp      %0, %1       \n"
    "beq      2f           \n"
    "strexb   %0, %1, [%2] \n"
    "cmp      %0, %1       \n"
    "beq      1b           \n"
    "dmb                   \n"
    "2:                    \n"
    : "+r" (ret)
    : "r" (UP_SP_LOCKED), "r" (lock)
    : "memory"
  );

  return ret;
}
#endif

/* See prototype in nuttx/include/nuttx/spinlock.h */

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_INCLUDE_SPINLOCK_H */

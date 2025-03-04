/****************************************************************************
 * arch/arm64/include/spinlock.h
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

#ifndef __ARCH_ARM64_INCLUDE_SPINLOCK_H
#define __ARCH_ARM64_INCLUDE_SPINLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif /* __ASSEMBLY__ */

#include <arch/barriers.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Spinlock states */

#define SP_UNLOCKED 0  /* The Un-locked state */
#define SP_LOCKED   1  /* The Locked state */

/* Memory barriers for use with NuttX spinlock logic
 *
 * Data Memory Barrier (DMB) acts as a memory barrier. It ensures that all
 * explicit memory accesses that appear in program order before the DMB
 * instruction are observed before any explicit memory accesses that appear
 * in program order after the DMB instruction. It does not affect the
 * ordering of any other instructions executing on the processor
 *
 *   dmb st - Data memory barrier.  Wait for stores to complete.
 *
 * Data Synchronization Barrier (DSB) acts as a special kind of memory
 * barrier. No instruction in program order after this instruction executes
 * until this instruction completes. This instruction completes when: (1) All
 * explicit memory accesses before this instruction complete, and (2) all
 * Cache, Branch predictor and TLB maintenance operations before this
 * instruction complete.
 *
 *   dsb sy - Data synchronization barrier.  Assures that the CPU waits until
 *            all memory accesses are complete
 */

#define UP_WFE() __asm__ __volatile__ ("wfe" : : : "memory")
#define UP_SEV() __asm__ __volatile__ ("sev" : : : "memory")

#ifndef __ASSEMBLY__

/* The Type of a spinlock.
 * ARM official document
 * ARM® Cortex®-A Series, Version: 1.0, Programmer’s Guide for ARMv8-A
 * ARM DEN0024A (ID050815)
 *
 * chapter 14.1.4 Synchronization
 *
 * The A64 instruction set has instructions for implementing
 * synchronization functions:
 * -- Load Exclusive (LDXR): LDXR W|Xt, [Xn]
 * -- Store Exclusive (STXR): STXR Ws, W|Xt, [Xn] where Ws
 *     indicates whether the store completed successfully.
 *     0 = success.
 * -- Clear Exclusive access monitor (CLREX) This is used to
 *     clear the state of the Local Exclusive Monitor.
 */

typedef uint64_t spinlock_t;

#if defined(CONFIG_ARCH_HAVE_TESTSET)
static inline_function spinlock_t up_testset(volatile spinlock_t *lock)
{
  spinlock_t ret = SP_LOCKED;

  __asm__ __volatile__
  (
    "1:                     \n"
    "ldaxr    %0, [%2]      \n"
    "cmp      %0, %1        \n"
    "beq      2f            \n"
    "stxr     %w0, %1, [%2] \n"
    "cbnz     %w0, 1b       \n"
    "2:                     \n"
    : "+r" (ret)
    :  "r" (SP_LOCKED), "r" (lock)
    : "memory"
  );

  return ret;
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_INCLUDE_SPINLOCK_H */

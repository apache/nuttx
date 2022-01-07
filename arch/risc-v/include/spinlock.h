/****************************************************************************
 * arch/risc-v/include/spinlock.h
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

#ifndef __ARCH_RISCV_INCLUDE_SPINLOCK_H
#define __ARCH_RISCV_INCLUDE_SPINLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif /* __ASSEMBLY__ */

/* Include RISC-V architecture-specific IRQ definitions (including register
 * save structure and up_irq_save()/up_irq_restore() functions)
 */

/****************************************************************************
 * Pre-processor Definitions
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
 * Data Synchronization Barrier (DSB) acts as a special kind of memory
 * barrier. No instruction in program order after this instruction executes
 * until this instruction completes. This instruction completes when: (1) All
 * explicit memory accesses before this instruction complete, and (2) all
 * Cache, Branch predictor and TLB maintenance operations before this
 * instruction complete.
 *
 */

#define SP_DSB(n) __asm__ __volatile__ ("fence")
#define SP_DMB(n) __asm__ __volatile__ ("fence")

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* The Type of a spinlock.
 *
 * RISC-V architecture (in the standard atomic-instruction extension "A")
 * supports exclusive accesses to memory locations in the form of the
 * Load-Reserved (LR) and Store-Conditional (SC) instructions. RV64 supports
 * doubleword aligned data only but others supports word aligned data.
 *
 * RISC-V architecture supports fence instruction to ensure memory ordering
 */

typedef uintptr_t spinlock_t;

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

/* See prototype in nuttx/include/nuttx/spinlock.h */

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_INCLUDE_SPINLOCK_H */

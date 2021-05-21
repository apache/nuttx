/****************************************************************************
 * arch/risc-v/include/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_RISCV_INCLUDE_IRQ_H
#define __ARCH_RISCV_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <stdint.h>
#include <nuttx/irq.h>
#include <arch/csr.h>
#include <arch/chip/irq.h>

/* Include RISC-V architecture-specific IRQ definitions */

#if defined(CONFIG_ARCH_RV32IM) || defined(CONFIG_ARCH_RV32I)
#  include <arch/rv32im/irq.h>
#endif

#if defined(CONFIG_ARCH_RV64GC)
#  include <arch/rv64gc/irq.h>
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Disable interrupts and return the previous value of the mstatus register
 *
 ****************************************************************************/

static inline irqstate_t up_irq_save(void)
{
  irqstate_t flags;

  /* Read mstatus & clear machine interrupt enable (MIE) in mstatus */

  __asm__ __volatile__
    (
      "csrrc %0, mstatus, %1\n"
      : "=r" (flags)
      : "r"(MSTATUS_MIE)
      : "memory"
    );

  /* Return the previous mstatus value so that it can be restored with
   * up_irq_restore().
   */

  return flags;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore the value of the mstatus register
 *
 ****************************************************************************/

static inline void up_irq_restore(irqstate_t flags)
{
  __asm__ __volatile__
    (
      "csrw mstatus, %0\n"
      : /* no output */
      : "r" (flags)
      : "memory"
    );
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt state and enable interrupts
 *
 ****************************************************************************/

EXTERN irqstate_t up_irq_enable(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_RISCV_INCLUDE_IRQ_H */

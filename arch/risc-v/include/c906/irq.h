/****************************************************************************
 * arch/risc-v/include/c906/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_C906_IRQ_H
#define __ARCH_RISCV_INCLUDE_C906_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

/* IRQ 0-15 : (exception:interrupt=0) */

#define C906_IRQ_IAMISALIGNED  (0) /* Instruction Address Misaligned */
#define C906_IRQ_IAFAULT       (1) /* Instruction Address Fault */
#define C906_IRQ_IINSTRUCTION  (2) /* Illegal Instruction */
#define C906_IRQ_BPOINT        (3) /* Break Point */
#define C906_IRQ_LAMISALIGNED  (4) /* Load Address Misaligned */
#define C906_IRQ_LAFAULT       (5) /* Load Access Fault */
#define C906_IRQ_SAMISALIGNED  (6) /* Store/AMO Address Misaligned */
#define C906_IRQ_SAFAULT       (7) /* Store/AMO Access Fault */
#define C906_IRQ_ECALLU        (8) /* Environment Call from U-mode */

#define C906_IRQ_ECALLM       (11) /* Environment Call from M-mode */

/* IRQ 16- : (async event:interrupt=1) */

#define C906_IRQ_ASYNC        (16)
#define C906_IRQ_SSOFT    (C906_IRQ_ASYNC + 1)  /* Supervisor Software Int */
#define C906_IRQ_MSOFT    (C906_IRQ_ASYNC + 3)  /* Machine Software Int */
#define C906_IRQ_STIMER   (C906_IRQ_ASYNC + 5)  /* Supervisor Timer Int */
#define C906_IRQ_MTIMER   (C906_IRQ_ASYNC + 7)  /* Machine Timer Int */
#define C906_IRQ_SEXT     (C906_IRQ_ASYNC + 9)  /* Supervisor External Int */
#define C906_IRQ_MEXT     (C906_IRQ_ASYNC + 11) /* Machine External Int */
#define C906_IRQ_HPMOV    (C906_IRQ_ASYNC + 17) /* HPM Overflow Int */

/* Machine Global External Interrupt */

#define C906_IRQ_PERI_START   (C906_IRQ_ASYNC + 18)

#ifdef CONFIG_C906_WITH_QEMU
#define C906_IRQ_UART0        (C906_IRQ_PERI_START + 32)
#else
#define C906_IRQ_UART0        (C906_IRQ_PERI_START + 32)
#endif

/* Total number of IRQs */

#define NR_IRQS               (C906_IRQ_UART0 + 1)

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
 * Public Function Prototypes
 ****************************************************************************/

EXTERN irqstate_t  up_irq_save(void);
EXTERN void up_irq_restore(irqstate_t);
EXTERN irqstate_t up_irq_enable(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_INCLUDE_C906_IRQ_H */

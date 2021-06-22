/****************************************************************************
 * arch/risc-v/include/jh7100/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_JH7100_IRQ_H
#define __ARCH_RISCV_INCLUDE_JH7100_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

/* IRQ 0-15 : (exception:interrupt=0) */

#define JH7100_IRQ_IAMISALIGNED  (0) /* Instruction Address Misaligned */
#define JH7100_IRQ_IAFAULT       (1) /* Instruction Address Fault */
#define JH7100_IRQ_IINSTRUCTION  (2) /* Illegal Instruction */
#define JH7100_IRQ_BPOINT        (3) /* Break Point */
#define JH7100_IRQ_LAMISALIGNED  (4) /* Load Address Misaligned */
#define JH7100_IRQ_LAFAULT       (5) /* Load Access Fault */
#define JH7100_IRQ_SAMISALIGNED  (6) /* Store/AMO Address Misaligned */
#define JH7100_IRQ_SAFAULT       (7) /* Store/AMO Access Fault */
#define JH7100_IRQ_ECALLU        (8) /* Environment Call from U-mode */

#define JH7100_IRQ_ECALLM       (11) /* Environment Call from M-mode */

/* IRQ 16- : (async event:interrupt=1) */

#define JH7100_IRQ_ASYNC        (16)
//#define JH7100_IRQ_SSOFT    (JH7100_IRQ_ASYNC + 1)  /* Supervisor Software Int */
#define JH7100_IRQ_MSOFT    (JH7100_IRQ_ASYNC + 3)  /* Machine Software Int */
//#define JH7100_IRQ_STIMER   (JH7100_IRQ_ASYNC + 5)  /* Supervisor Timer Int */
#define JH7100_IRQ_MTIMER   (JH7100_IRQ_ASYNC + 7)  /* Machine Timer Int */
//#define JH7100_IRQ_SEXT     (JH7100_IRQ_ASYNC + 9)  /* Supervisor External Int */
#define JH7100_IRQ_MEXT     (JH7100_IRQ_ASYNC + 11) /* Machine External Int */
//#define JH7100_IRQ_HPMOV    (JH7100_IRQ_ASYNC + 17) /* HPM Overflow Int */

/* Machine Global External Interrupt */

#define JH7100_IRQ_PERI_START   (JH7100_IRQ_ASYNC + 18)

#ifdef CONFIG_JH7100_WITH_QEMU
#define JH7100_IRQ_UART0        (JH7100_IRQ_PERI_START + 32)
#else
//#define JH7100_IRQ_UART0        (JH7100_IRQ_PERI_START + 32)
#define JH7100_IRQ_UART0        (JH7100_IRQ_PERI_START + 73) // Manually extracted from Device Tree.
#endif

/* Total number of IRQs */

#define NR_IRQS               (JH7100_IRQ_UART0 + 4) // RJL FIXME: made up.

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
#endif /* __ARCH_RISCV_INCLUDE_JH7100_IRQ_H */

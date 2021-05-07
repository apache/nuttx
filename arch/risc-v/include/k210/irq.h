/****************************************************************************
 * arch/risc-v/include/k210/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_K210_IRQ_H
#define __ARCH_RISCV_INCLUDE_K210_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

/* IRQ 0-15 : (exception:interrupt=0) */

#define K210_IRQ_IAMISALIGNED  (0) /* Instruction Address Misaligned */
#define K210_IRQ_IAFAULT       (1) /* Instruction Address Fault */
#define K210_IRQ_IINSTRUCTION  (2) /* Illegal Instruction */
#define K210_IRQ_BPOINT        (3) /* Break Point */
#define K210_IRQ_LAMISALIGNED  (4) /* Load Address Misaligned */
#define K210_IRQ_LAFAULT       (5) /* Load Access Fault */
#define K210_IRQ_SAMISALIGNED  (6) /* Store/AMO Address Misaligned */
#define K210_IRQ_SAFAULT       (7) /* Store/AMO Access Fault */
#define K210_IRQ_ECALLU        (8) /* Environment Call from U-mode */
                                   /* 9-10: Reserved */

#define K210_IRQ_ECALLM       (11) /* Environment Call from M-mode */
                                   /* 12-15: Reserved */

/* IRQ 16- : (async event:interrupt=1) */

#define K210_IRQ_ASYNC        (16)
#define K210_IRQ_MSOFT    (K210_IRQ_ASYNC + 3)  /* Machine Software Int */
#define K210_IRQ_MTIMER   (K210_IRQ_ASYNC + 7)  /* Machine Timer Int */
#define K210_IRQ_MEXT     (K210_IRQ_ASYNC + 11) /* Machine External Int */

/* Machine Global External Interrupt */

#ifdef CONFIG_K210_WITH_QEMU
#define K210_IRQ_UART0    (K210_IRQ_MEXT + 4)
#else
#define K210_IRQ_UART0    (K210_IRQ_MEXT + 33)
#endif

/* Total number of IRQs */

#define NR_IRQS            (K210_IRQ_UART0 + 1)

#endif /* __ARCH_RISCV_INCLUDE_K210_IRQ_H */

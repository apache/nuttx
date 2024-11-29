/****************************************************************************
 * arch/risc-v/include/k230/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_K230_IRQ_H
#define __ARCH_RISCV_INCLUDE_K230_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

#ifndef CONFIG_BUILD_KERNEL
#  define K230_IRQ_TIMER  (RISCV_IRQ_MTIMER)
#  define K230_IRQ_UART0  (RISCV_IRQ_MEXT + 16)
#else
#  define K230_IRQ_TIMER  (RISCV_IRQ_STIMER)
#  define K230_IRQ_UART0  (RISCV_IRQ_SEXT + 16)
#endif

#define K230_IRQ_UART3    (K230_IRQ_UART0 + 3)
#define K230_IRQ_IPI0     (K230_IRQ_UART0 + 93)
#define K230_IRQ_IPI3     (K230_IRQ_IPI0 + 3)

#define K230_PLIC_IRQS    208

/* NR_IRQS is needed by NuttX */

#define NR_IRQS           (K230_IRQ_IPI3 + 1)

#endif /* __ARCH_RISCV_INCLUDE_K230_IRQ_H */

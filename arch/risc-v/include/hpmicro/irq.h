/****************************************************************************
 * arch/risc-v/include/hpmicro/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_HPMICRO_IRQ_H
#define __ARCH_RISCV_INCLUDE_HPMICRO_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */
#define HPM_IRQ_PERI_START   (RISCV_IRQ_ASYNC + 20)

/* Machine Global External Interrupt */

/* Total number of IRQs */
#define HPM6750_NR_IRQS      127
#define HPM6360_NR_IRQS      78

#define NR_IRQS              (HPM_IRQ_PERI_START + HPM6750_NR_IRQS)

#endif /* __ARCH_RISCV_INCLUDE_HPMICRO_IRQ_H */

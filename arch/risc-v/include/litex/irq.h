/****************************************************************************
 * arch/risc-v/include/litex/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_LITEX_IRQ_H
#define __ARCH_RISCV_INCLUDE_LITEX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

#define LITEX_IRQ_UART0    (RISCV_IRQ_MEXT + 1)
#define LITEX_IRQ_TIMER0   (RISCV_IRQ_MEXT + 2)

/* Total number of IRQs */

#define NR_IRQS            (LITEX_IRQ_TIMER0 + 1)

#endif /* __ARCH_RISCV_INCLUDE_LITEX_IRQ_H */

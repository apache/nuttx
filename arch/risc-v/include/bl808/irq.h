/****************************************************************************
 * arch/risc-v/include/bl808/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_BL808_IRQ_H
#define __ARCH_RISCV_INCLUDE_BL808_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

#define NR_IRQS (RISCV_IRQ_SEXT + 57)

#define BL808_IRQ_NUM_BASE (16)

#define BL808_IRQ_UART3 (RISCV_IRQ_SEXT + BL808_IRQ_NUM_BASE + 4)

#endif /* __ARCH_RISCV_INCLUDE_BL808_IRQ_H */

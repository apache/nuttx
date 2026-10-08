/****************************************************************************
 * arch/risc-v/include/erbium/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_ERBIUM_IRQ_H
#define __ARCH_RISCV_INCLUDE_ERBIUM_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ET-Minion reports a 5-bit exception code. Besides the standard codes it
 * raises 25 (instruction bus error), 26 (instruction ECC error), 27 and 28
 * (load and store split-page fault) and 30 (M-code emulation: the
 * instruction has no hardware implementation and traps to machine mode).
 * Number all 32 codes as exceptions so that none of them is dispatched as
 * an interrupt. Interrupt IRQ numbers therefore start at 32.
 */

#define RISCV_MAX_EXCEPTION   (31)

#define ERBIUM_IRQ_IBUSERR    (25)  /* Instruction bus error */
#define ERBIUM_IRQ_IECCERR    (26)  /* Instruction ECC error */
#define ERBIUM_IRQ_LSPLITPF   (27)  /* Load split-page fault */
#define ERBIUM_IRQ_SSPLITPF   (28)  /* Store split-page fault */
#define ERBIUM_IRQ_MCODE      (30)  /* M-code emulation */

/* Reason strings for the codes after the common table. The common table
 * names codes 16-19 itself when Machine ISA 1.13 support is enabled.
 */

#ifdef CONFIG_ARCH_RV_MACHINE_ISA_1_13
#  define ERBIUM_FIRST_CUSTOM_EXCEPTION (20)
#  define ERBIUM_EXCEPTION_REASONS_16_19
#else
#  define ERBIUM_FIRST_CUSTOM_EXCEPTION (16)
#  define ERBIUM_EXCEPTION_REASONS_16_19 \
     "Reserved", "Reserved", "Reserved", "Reserved",
#endif

#define RISCV_CUSTOM_EXCEPTION_REASONS                         \
  ERBIUM_EXCEPTION_REASONS_16_19                               \
  "Reserved", "Reserved", "Reserved", "Reserved", "Reserved",  \
  "Instruction bus error", "Instruction ECC error",            \
  "Load split-page fault", "Store split-page fault",           \
  "Reserved", "M-code emulation", "Reserved",

#define ERBIUM_IRQ_UART0      (RISCV_IRQ_MEXT + 3)
#define ERBIUM_IRQ_EXT_MAX    (RISCV_IRQ_MEXT + 6)

#define NR_IRQS               (ERBIUM_IRQ_EXT_MAX + 1)

#endif /* __ARCH_RISCV_INCLUDE_ERBIUM_IRQ_H */

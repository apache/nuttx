/****************************************************************************
 * arch/risc-v/src/erbium/erbium_mcode.h
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

#ifndef __ARCH_RISCV_SRC_ERBIUM_ERBIUM_MCODE_H
#define __ARCH_RISCV_SRC_ERBIUM_ERBIUM_MCODE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/irq.h>

#ifdef CONFIG_ERBIUM_MCODE_EMULATION

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: erbium_mcode_emulate
 *
 * Description:
 *   Perform the instruction that raised exception 30 (M-code emulation)
 *   on the saved context and advance its EPC. Runs in trap context with
 *   interrupts disabled and uses no OS service.
 *
 * Input Parameters:
 *   regs - Saved registers of the trapped context.
 *
 * Returned Value:
 *   OK if the instruction was emulated; -EINVAL if it must raise
 *   illegal-instruction instead (FPU off or reserved rounding mode);
 *   -ENOSYS if it is not emulated.
 *
 ****************************************************************************/

int erbium_mcode_emulate(uintreg_t *regs);

/* Live FP register access (erbium_mcode_fpr.S). The trap path does not
 * save the F registers, so the emulator reads and writes them in place.
 */

uint32_t erbium_mcode_getfpr(unsigned int reg);
void erbium_mcode_setfpr(unsigned int reg, uint32_t val);

#endif /* CONFIG_ERBIUM_MCODE_EMULATION */
#endif /* __ARCH_RISCV_SRC_ERBIUM_ERBIUM_MCODE_H */

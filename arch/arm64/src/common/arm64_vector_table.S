/****************************************************************************
 * arch/arm64/src/common/arm64_vector_table.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm64_macro.inc"
#include "arch/irq.h"
#include "arm64_fatal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "arm64_vector_table.S"

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/* Save Corruptible Registers and exception context
 * on the task stack
 * note: allocate stackframe with XCPTCONTEXT_GP_REGS
 *     which is ARM64_ESF_REGS + ARM64_CS_REGS
 *     but only save ARM64_ESF_REGS
 */

.macro arm64_enter_exception xreg0, xreg1
    sub    sp, sp, #8 * XCPTCONTEXT_GP_REGS

    stp    x0,  x1,  [sp, #8 * REG_X0]
    stp    x2,  x3,  [sp, #8 * REG_X2]
    stp    x4,  x5,  [sp, #8 * REG_X4]
    stp    x6,  x7,  [sp, #8 * REG_X6]
    stp    x8,  x9,  [sp, #8 * REG_X8]
    stp    x10, x11, [sp, #8 * REG_X10]
    stp    x12, x13, [sp, #8 * REG_X12]
    stp    x14, x15, [sp, #8 * REG_X14]
    stp    x16, x17, [sp, #8 * REG_X16]
    stp    x18, x19, [sp, #8 * REG_X18]
    stp    x20, x21, [sp, #8 * REG_X20]
    stp    x22, x23, [sp, #8 * REG_X22]
    stp    x24, x25, [sp, #8 * REG_X24]
    stp    x26, x27, [sp, #8 * REG_X26]
    stp    x28, x29, [sp, #8 * REG_X28]

    /* Save the current task's SP_ELx and x30 */
    mov    \xreg0, sp
    stp    x30, \xreg0,  [sp, #8 * REG_X30]

    /* ELR and SPSR */
    mrs    \xreg0, elr_el1
    mrs    \xreg1, spsr_el1
    stp    \xreg0, \xreg1,  [sp, #8 * REG_ELR]

    /* increment exception depth */

    mrs    \xreg0, tpidrro_el0
    mov    \xreg1, #1
    add    \xreg0, \xreg0, \xreg1
    msr    tpidrro_el0, \xreg0

#ifdef CONFIG_ARCH_FPU
    bl    arm64_fpu_enter_exception
#endif

.endm

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Four types of exceptions:
 * - synchronous: aborts from MMU, SP/CP alignment checking, unallocated
 *   instructions, SVCs/SMCs/HVCs, ...)
 * - IRQ: group 1 (normal) interrupts
 * - FIQ: group 0 or secure interrupts
 * - SError: fatal system errors
 *
 * Four different contexts:
 * - from same exception level, when using the SP_EL0 stack pointer
 * - from same exception level, when using the SP_ELx stack pointer
 * - from lower exception level, when this is AArch64
 * - from lower exception level, when this is AArch32
 *
 * +------------------+------------------+-------------------------+
 * |     Address      |  Exception type  |       Description       |
 * +------------------+------------------+-------------------------+
 * | VBAR_ELn + 0x000 | Synchronous      | Current EL with SP0     |
 * |          + 0x080 | IRQ / vIRQ       |                         |
 * |          + 0x100 | FIQ / vFIQ       |                         |
 * |          + 0x180 | SError / vSError |                         |
 * +------------------+------------------+-------------------------+
 * |          + 0x200 | Synchronous      | Current EL with SPx     |
 * |          + 0x280 | IRQ / vIRQ       |                         |
 * |          + 0x300 | FIQ / vFIQ       |                         |
 * |          + 0x380 | SError / vSError |                         |
 * +------------------+------------------+-------------------------+
 * |          + 0x400 | Synchronous      | Lower EL using  AArch64 |
 * |          + 0x480 | IRQ / vIRQ       |                         |
 * |          + 0x500 | FIQ / vFIQ       |                         |
 * |          + 0x580 | SError / vSError |                         |
 * +------------------+------------------+-------------------------+
 * |          + 0x600 | Synchronous      | Lower EL using AArch64  |
 * |          + 0x680 | IRQ / vIRQ       |                         |
 * |          + 0x700 | FIQ / vFIQ       |                         |
 * |          + 0x780 | SError / vSError |                         |
 * +------------------+------------------+-------------------------+
 */

GTEXT(_vector_table)

    /* The whole table must be 2K aligned */

    .align 11
SECTION_SUBSEC_FUNC(exc_vector_table,_vector_table_section,_vector_table)

    /* Current EL with SP0 / Synchronous */

    .align 7
    arm64_enter_exception x0, x1
    b    arm64_sync_exc

    /* Current EL with SP0 / IRQ */

    .align 7
    arm64_enter_exception x0, x1
    b    arm64_irq_handler

    /* Current EL with SP0 / FIQ */

    .align 7
    arm64_enter_exception x0, x1
    b     arm64_irq_spurious

    /* Current EL with SP0 / SError */

    .align 7
    arm64_enter_exception x0, x1
    b    arm64_serror_handler

    /* Current EL with SPx / Synchronous */

    .align 7
    arm64_enter_exception x0, x1
    b    arm64_sync_exc

    /* Current EL with SPx / IRQ */

    .align 7
    arm64_enter_exception x0, x1
    b    arm64_irq_handler

    /* Current EL with SPx / FIQ */

    .align 7
    arm64_enter_exception x0, x1
    b     arm64_irq_spurious

    /* Current EL with SPx / SError */

    .align 7
    arm64_enter_exception x0, x1
    b    arm64_serror_handler

    /* Lower EL using AArch64 / Synchronous */

    .align 7
    arm64_enter_exception x0, x1
    b    arm64_sync_exc

    /* Lower EL using AArch64 / IRQ */

    .align 7
    arm64_enter_exception x0, x1
    b    arm64_irq_handler

    /* Lower EL using AArch64 / FIQ */

    .align 7
    arm64_enter_exception x0, x1
    b     arm64_irq_spurious

    /* Lower EL using AArch64 / SError */

    .align 7
    arm64_enter_exception x0, x1
    b    arm64_serror_handler

    /* Lower EL using AArch32 / Synchronous */

    .align 7
    arm64_enter_exception x0, x1
    b    arm64_mode32_error

    /* Lower EL using AArch32 / IRQ */

    .align 7
    arm64_enter_exception x0, x1
    b    arm64_mode32_error

    /* Lower EL using AArch32 / FIQ */

    .align 7
    arm64_enter_exception x0, x1
    b    arm64_mode32_error

    /* Lower EL using AArch32 / SError */

    .align 7
    arm64_enter_exception x0, x1
    b    arm64_mode32_error

/* Restore Corruptible Registers and exception context
 * from the task stack.
 */

GTEXT(arm64_exit_exception)
SECTION_FUNC(text, arm64_exit_exception)
#ifdef CONFIG_ARCH_FPU
    bl    arm64_fpu_exit_exception
GTEXT(arm64_exit_exc_fpu_done)
arm64_exit_exc_fpu_done:
#endif

    /* restore spsr and elr at el1*/

    ldp    x0, x1,   [sp, #8 * REG_ELR]
    msr    elr_el1,   x0
    msr    spsr_el1,  x1

    /* decrement exception depth */

    mrs    x0, tpidrro_el0
    mov    x1, #1
    sub    x0, x0, x1
    msr    tpidrro_el0, x0

    ldp    x30, x0,  [sp, #8 * REG_X30]
    ldp    x0,  x1,  [sp, #8 * REG_X0]
    ldp    x2,  x3,  [sp, #8 * REG_X2]
    ldp    x4,  x5,  [sp, #8 * REG_X4]
    ldp    x6,  x7,  [sp, #8 * REG_X6]
    ldp    x8,  x9,  [sp, #8 * REG_X8]
    ldp    x10, x11, [sp, #8 * REG_X10]
    ldp    x12, x13, [sp, #8 * REG_X12]
    ldp    x14, x15, [sp, #8 * REG_X14]
    ldp    x16, x17, [sp, #8 * REG_X16]
    ldp    x18, x19, [sp, #8 * REG_X18]
    ldp    x20, x21, [sp, #8 * REG_X20]
    ldp    x22, x23, [sp, #8 * REG_X22]
    ldp    x24, x25, [sp, #8 * REG_X24]
    ldp    x26, x27, [sp, #8 * REG_X26]
    ldp    x28, x29, [sp, #8 * REG_X28]

    add    sp, sp, #8 * XCPTCONTEXT_GP_REGS

    eret

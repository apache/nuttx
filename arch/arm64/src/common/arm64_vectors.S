/****************************************************************************
 * arch/arm64/src/common/arm64_vectors.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/chip/chip.h>
#include "arch/syscall.h"
#include "arm64_macro.inc"
#include "arch/irq.h"
#include "arm64_fatal.h"
#include "arm64_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .file    "arm64_vectors.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: up_saveusercontext
 *
 * Description:
 *   Take a snapshot of the thread GP registers context
 *   x0 --- pointer to stack context
 *
 ****************************************************************************/
GTEXT(up_saveusercontext)
SECTION_FUNC(text, up_saveusercontext)

    stp    x0,  x1,  [x0, #8 * REG_X0]
    stp    x2,  x3,  [x0, #8 * REG_X2]
    stp    x4,  x5,  [x0, #8 * REG_X4]
    stp    x6,  x7,  [x0, #8 * REG_X6]
    stp    x8,  x9,  [x0, #8 * REG_X8]
    stp    x10, x11, [x0, #8 * REG_X10]
    stp    x12, x13, [x0, #8 * REG_X12]
    stp    x14, x15, [x0, #8 * REG_X14]
    stp    x16, x17, [x0, #8 * REG_X16]
    stp    x18, x19, [x0, #8 * REG_X18]
    stp    x20, x21, [x0, #8 * REG_X20]
    stp    x22, x23, [x0, #8 * REG_X22]
    stp    x24, x25, [x0, #8 * REG_X24]
    stp    x26, x27, [x0, #8 * REG_X26]
    stp    x28, x29, [x0, #8 * REG_X28]

    /* Save the current task's SP_ELx and x30 */
    mov    x4,  sp
    stp    x30, x4,  [x0, #8 * REG_X30]

    /* ELR and SPSR */
#if CONFIG_ARCH_ARM64_EXCEPTION_LEVEL == 3
    mrs    x4,  elr_el3
    mrs    x5,  spsr_el3
#else
    mrs    x4,  elr_el1
    mrs    x5,  spsr_el1
#endif
    stp    x4,  x5,  [x0, #8 * REG_ELR]

    mrs    x4,  sctlr_el1
    str    x4,  [x0, #8 * REG_SCTLR_EL1]

    ret

/****************************************************************************
 * Function: arm64_jump_to_user
 *
 * Description:
 *  Routine to jump to user space, called when a user process is started and
 *  the kernel is ready to give control to the user task in user space.
 *
 * arm64_jump_to_user(entry, x0, x1, regs)
 *     entry:  process entry point
 *     x0:     parameter 0 for process
 *     x1:     parameter 1 for process
 *     sp_el0: user stack pointer
 *     regs:   integer register save area to use
 *
 ****************************************************************************/

#ifndef CONFIG_BUILD_FLAT
GTEXT(arm64_jump_to_user)
SECTION_FUNC(text, arm64_jump_to_user)
    msr daifset, #IRQ_DAIF_MASK
    mov sp,  x4
    str x0,  [sp, #8 * REG_ELR]
    str x1,  [sp, #8 * REG_X0]
    str x2,  [sp, #8 * REG_X1]
    str x3,  [sp, #8 * REG_SP_EL0]
    mrs x0,  spsr_el1
    and x0,  x0, #~SPSR_MODE_MASK
    #orr x0, x0, #SPSR_MODE_EL0T # EL0T=0x00, out of range for orr
    str x0,  [sp, #8 * REG_SPSR]
    mrs x0,  sctlr_el1
    str x0,  [sp, #8 * REG_SCTLR_EL1]
    b arm64_exit_exception
#endif

/****************************************************************************
 * Function: arm64_sync_exc
 *
 * Description:
 *   handle synchronous exception for AArch64
 *
 ****************************************************************************/

GTEXT(arm64_sync_exc)
SECTION_FUNC(text, arm64_sync_exc)
    /* checking the EC value to see which exception need to be handle */

#if CONFIG_ARCH_ARM64_EXCEPTION_LEVEL == 3
    mrs    x9, esr_el3
#else
    mrs    x9, esr_el1
#endif
    lsr    x10, x9, #26

    /* 0x15 = SVC system call */

    cmp    x10, #0x15

    /* if this is a svc call ?*/

    bne    2f

#ifdef CONFIG_LIB_SYSCALL
    /* Handle user system calls separately */

    cmp    x0, #CONFIG_SYS_RESERVED
    blt    reserved_syscall

    /* Call dispatch_syscall() on the kernel stack with interrupts enabled */

    mrs    x10, spsr_el1
    and    x10, x10, #IRQ_SPSR_MASK
    cmp    x10, xzr
    bne    1f
    msr    daifclr, #IRQ_DAIF_MASK /* Re-enable interrupts */

1:
    bl     dispatch_syscall
    msr    daifset, #IRQ_DAIF_MASK /* Disable interrupts */

    /* Save the return value into the user context */

    str    x0, [sp, #8 * REG_X0]

    /* Return from exception */

    b      arm64_exit_exception

reserved_syscall:
#endif

    /* Switch to IRQ stack and save current sp on it. */
#ifdef CONFIG_SMP
    get_cpu_id x0
    ldr    x1, =(g_cpu_int_stacktop)
    lsl    x0, x0, #3
    ldr    x1, [x1, x0]
#else
    ldr    x1, =(g_interrupt_stack + CONFIG_ARCH_INTERRUPTSTACK)
#endif

    mov    x0, sp
    mov    sp, x1

    bl     arm64_syscall        /* Call the handler */

    mov    sp, x0
    b      arm64_exit_exception
2:
    mov    x0, sp
    adrp   x5, arm64_fatal_handler
    add    x5, x5, #:lo12:arm64_fatal_handler
    br     x5

/****************************************************************************
 * Name: arm64_irq_handler
 *
 * Description:
 *   Interrupt exception handler
 *
 ****************************************************************************/

GTEXT(arm64_irq_handler)
SECTION_FUNC(text, arm64_irq_handler)
    /* Switch to IRQ stack and save current sp on it. */
#ifdef CONFIG_SMP
    get_cpu_id x0
    ldr    x1, =(g_cpu_int_stacktop)
    lsl    x0, x0, #3
    ldr    x1, [x1, x0]
#else
    ldr    x1, =(g_interrupt_stack + CONFIG_ARCH_INTERRUPTSTACK)
#endif

    mov    x0, sp
    mov    sp, x1

    /* Call arm64_decodeirq() on the interrupt stack
     * with interrupts disabled
     */

    bl     arm64_decodeirq

    mov    sp, x0
    b      arm64_exit_exception

/****************************************************************************
 * Name: arm64_serror_handler
 *
 * Description:
 *   SError exception handler
 *
 ****************************************************************************/

GTEXT(arm64_serror_handler)
SECTION_FUNC(text, arm64_serror_handler)
    mov    x0, sp
    adrp   x5, arm64_fatal_handler
    add    x5, x5, #:lo12:arm64_fatal_handler
    br     x5
    /* Return here only in case of recoverable error */

    b      arm64_exit_exception

/****************************************************************************
 * Name: arm64_mode32_handler
 *
 * Description:
 *   Mode32 exception handler
 *
 ****************************************************************************/

GTEXT(arm64_mode32_handler)
SECTION_FUNC(text, arm64_mode32_handler)
    mov    x0, sp
    adrp   x5, arm64_fatal_handler
    add    x5, x5, #:lo12:arm64_fatal_handler
    br     x5
    /* Return here only in case of recoverable error */

    b      arm64_exit_exception

/****************************************************************************
 * Name: arm64_fiq_handler
 *
 * Description:
 *   Interrupt exception handler
 *
 ****************************************************************************/

GTEXT(arm64_fiq_handler)
SECTION_FUNC(text, arm64_fiq_handler)
#ifndef CONFIG_ARM64_DECODEFIQ

    mov    x0, sp
    adrp   x5, arm64_fatal_handler
    add    x5, x5, #:lo12:arm64_fatal_handler
    br     x5

    /* Return here only in case of recoverable error */

    b      arm64_exit_exception
#else
   /* Switch to FIQ stack and save current sp on it. */
#ifdef CONFIG_SMP
    get_cpu_id x0
    ldr    x1, =(g_cpu_int_fiq_stacktop)
    lsl    x0, x0, #3
    ldr    x1, [x1, x0]
#else
    ldr    x1, =(g_interrupt_fiq_stack + CONFIG_ARCH_INTERRUPTSTACK)
#endif

    mov    x0, sp
    mov    sp, x1

    /* Call arm64_decodeirq() on the interrupt stack
     * with interrupts disabled
     */

    bl     arm64_decodeirq

    mov    sp, x0
    b      arm64_exit_exception
#endif

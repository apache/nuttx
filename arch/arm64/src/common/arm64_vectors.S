/****************************************************************************
 * arch/arm64/src/common/arm64_vectors.S
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
 * Assembly Macros
 ****************************************************************************/

.macro arm64_exception_context_save xreg0, xreg1 xfp

    /* Save the current task's SP_EL0 and exception depth */
    mrs    \xreg0, sp_el0
    mrs    \xreg1, tpidrro_el0
    stp    \xreg0, \xreg1, [\xfp, #8 * REG_SP_EL0]

.endm

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: up_saveusercontext
 *
 * Description:
 *   Take a snapshot of the thread GP registers context
 *   x0 --- pointer to struct regs_context
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

    arm64_exception_context_save x4 x5 x0

    ret

/****************************************************************************
 * Function: arm64_jump_to_user
 *
 * Description:
 *  Routine to jump to user space, called when a user process is started and
 *  the kernel is ready to give control to the user task in user space.
 *
 * arm64_jump_to_user(entry, x0, x1, regs)
 *     entry: process entry point
 *     x0:    parameter 0 for process
 *     x1:    parameter 1 for process
 *     regs:  integer register save area to use
 *
 ****************************************************************************/

#ifndef CONFIG_BUILD_FLAT
GTEXT(arm64_jump_to_user)
SECTION_FUNC(text, arm64_jump_to_user)
    msr daifset, #IRQ_DAIF_MASK
    mov sp,  x3
    str x0,  [sp, #8 * REG_ELR]
    str x1,  [sp, #8 * REG_X0]
    str x2,  [sp, #8 * REG_X1]
    mrs x0,  spsr_el1
    and x0,  x0, #~SPSR_MODE_MASK
    #orr x0, x0, #SPSR_MODE_EL0T # EL0T=0x00, out of range for orr
    str x0,  [sp, #8 * REG_SPSR]
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
    mrs    x0, esr_el3
#else
    mrs    x0, esr_el1
#endif
    lsr    x1, x0, #26

#ifdef CONFIG_ARCH_FPU
    /* fpu trap */

    cmp    x1, #0x07 /*Access to SIMD or floating-point */
    bne    1f
    mov    x0, sp

    /* when the fpu trap is handled */

    b      arm64_exit_exception
1:
#endif
    /* 0x15 = SVC system call */

    cmp    x1, #0x15

    /* if this is a svc call ?*/

    bne    exc_handle

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

    /* x0 = syscall_cmd
     * if ( x0 <= SYS_switch_context ) {
     *     call context_switch
     *     it's a context switch syscall, so context need to be done
     * }
     * #define SYS_save_context          (0)
     * #define SYS_restore_context       (1)
     * #define SYS_switch_context        (2)
     */

    ldr    x0, [sp, #8 * REG_X0]
    cmp    x0, #SYS_save_context
    beq  save_context

    cmp    x0, #SYS_switch_context
    beq  context_switch

    cmp    x0, #SYS_restore_context
    beq  context_switch

    /* Normal syscall, thread context will not switch
     *
     * call the SVC handler with interrupts disabled.
     * void arm64_syscall(uint64_t *regs)
     * in:
     *      regs = pointer to struct reg_context allocating
     *         from stack, esf_reg has put on it
     *      regs[REG_X0]: syscall cmd
     *      regs[REG_X1] ~ regs[REG_X6]: syscall parameter
     * out:
     *      x0: return by arm64_syscall
     */

    mov    x0, sp /* x0 = reg frame */

    /* Call arm64_syscall() on the user stack */

    bl    arm64_syscall        /* Call the handler */

    /* Return from exception */

    b    arm64_exit_exception

context_switch:
    /* Call arm64_syscall_switch() for context switch
     *
     * uint64_t * arm64_syscall_switch(uint64_t * regs)
     * out:
     *      x0: return by arm64_syscall_switch, restore task context
     *      regs[REG_X1]: save task context, if x1 = 0, only restore x0
     */

    mov    x0, sp
    str    x0, [sp, #-16]!
    bl    arm64_syscall_switch

    /* get save task reg context pointer */

    ldr    x1, [sp], #16

#ifdef CONFIG_SMP
    /* Notes:
     * Complete any pending TLB or cache maintenance on this CPU in case
     * the thread migrates to a different CPU.
     * This full barrier is also required by the membarrier system
     * call.
     */

    dsb   ish
#endif

    b    arm64_exit_exception

save_context:
    arm64_exception_context_save x0 x1 sp

    mov    x0, sp
    bl    arm64_syscall_save_context

    /* Save the return value into the ESF */

    str    x0, [sp, #8 * REG_X0]

    /* Return from exception */

    b    arm64_exit_exception

exc_handle:
    mov    x0, sp

    /* void arm64_fatal_handler(struct regs_context * reg);
     * x0 = Exception stack frame
     */

    bl    arm64_fatal_handler

    /* Return here only in case of recoverable error */

    b    arm64_exit_exception

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
    get_cpu_id x1
    ldr    x0, =(g_cpu_int_stacktop)
    lsl    x1, x1, #3
    ldr    x0, [x0, x1]
#else
    ldr    x0, =(g_interrupt_stack + CONFIG_ARCH_INTERRUPTSTACK)
#endif
    /* Save the task's stack and switch irq stack */

    mov    x1, sp
    mov    sp, x0
    str    x1, [sp, #-16]!

    mov    x0, x1 /* x0 = reg frame */

    /* Call arm64_decodeirq() on the interrupt stack
     * with interrupts disabled
     */

    bl     arm64_decodeirq

    /* Upon return from arm64_decodeirq, x0 holds the pointer to the
     * call reg context area, which can be use to restore context.
     * This may or may not be the same value that was passed to arm64_decodeirq:
     * It will differ if a context switch is required.
     */

    ldr    x1, [sp], #16

#ifdef CONFIG_SMP
    /* Notes:
     * Complete any pending TLB or cache maintenance on this CPU in case
     * the thread migrates to a different CPU.
     * This full barrier is also required by the membarrier system
     * call.
     */
    dsb   ish

#endif

    b     arm64_exit_exception

/* TODO: if the arm64_fatal_handler return success, maybe need context switch */

GTEXT(arm64_serror_handler)
SECTION_FUNC(text, arm64_serror_handler)
    mov   x0, sp
    bl    arm64_fatal_handler
    /* Return here only in case of recoverable error */

    b    arm64_exit_exception

GTEXT(arm64_mode32_handler)
SECTION_FUNC(text, arm64_mode32_handler)
    mov    x0, sp
    bl    arm64_fatal_handler
    /* Return here only in case of recoverable error */

    b    arm64_exit_exception

GTEXT(arm64_fiq_handler)
SECTION_FUNC(text, arm64_fiq_handler)
#ifndef CONFIG_ARM64_DECODEFIQ

    mov    x0, sp
    bl    arm64_fatal_handler

    /* Return here only in case of recoverable error */

    b    arm64_exit_exception
#else
   /* Switch to FIQ stack and save current sp on it. */
#ifdef CONFIG_SMP
    get_cpu_id x1
    ldr    x0, =(g_cpu_int_fiq_stacktop)
    lsl    x1, x1, #3
    ldr    x0, [x0, x1]
#else
    ldr    x0, =(g_interrupt_fiq_stack + CONFIG_ARCH_INTERRUPTSTACK)
#endif
    /* Save the task's stack and switch fiq stack */

    mov    x1, sp
    mov    sp, x0
    str    x1, [sp, #-16]!

    mov    x0, x1 /* x0 = reg frame */

    /* Call arm64_decodefiq() on the interrupt stack
     * with interrupts disabled
     */

    bl     arm64_decodefiq

    /* Upon return from arm64_decodefiq, x0 holds the pointer to the
     * call reg context area, which can be use to restore context.
     * This may or may not be the same value that was passed to arm64_decodefiq:
     * It will differ if a context switch is required.
     */

    ldr    x1, [sp], #16

#ifdef CONFIG_SMP
    /* Notes:
     * Complete any pending TLB or cache maintenance on this CPU in case
     * the thread migrates to a different CPU.
     * This full barrier is also required by the membarrier system
     * call.
     */
    dsb   ish

#endif

    b     arm64_exit_exception
#endif

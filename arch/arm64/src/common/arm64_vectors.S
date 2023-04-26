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

    /* Save the TPIDR0/TPIDR1, which is the current tcb */

    mrs    \xreg0, tpidr_el0
    mrs    \xreg1, tpidr_el1
    stp    \xreg0, \xreg1, [\xfp, #8 * REG_TPIDR_EL0]

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
    str    x0, [sp, #-16]!

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
    mrs    x4,  elr_el1
    mrs    x5,  spsr_el1
    stp    x4,  x5,  [x0, #8 * REG_ELR]

    arm64_exception_context_save x4 x5 x0

    ldr    x0, [sp], #16

    ret

/****************************************************************************
 * Function: arm64_context_switch
 *
 * Description:
 *  Routine to handle context switch
 *
 * arm64_context_switch( x0, x1)
 *     x0: restore thread stack context
 *     x1: save thread stack context
 * note:
 *     x1 = 0, only restore x0
 *
 ****************************************************************************/

GTEXT(arm64_context_switch)
SECTION_FUNC(text, arm64_context_switch)
    cmp    x1, #0x0
    beq    restore_new

#if defined(CONFIG_ARCH_FPU) && defined(CONFIG_SMP)
    stp    x0, x1, [sp, #-16]!
    stp    xzr, x30, [sp, #-16]!
    bl     arm64_fpu_context_save
    ldp    xzr, x30, [sp], #16
    ldp    x0, x1, [sp], #16
#endif

    /* Save the current SP_EL0 */
    mov    x4,  sp
    str    x4, [x1, #8 * REG_SP_ELX]

    /* Save the current task's SP_EL0 and exception depth */
    mrs    x4, sp_el0
    mrs    x5, tpidrro_el0
    stp    x4, x5, [x1, #8 * REG_SP_EL0]

    /* Save the TPIDR0/TPIDR1, which is the current tcb */

    mrs    x4, tpidr_el0
    mrs    x5, tpidr_el1
    stp    x4, x5, [x1, #8 * REG_TPIDR_EL0]

restore_new:

    /* Restore SP_EL0 and thread's exception dept */
    ldp    x4,  x5,  [x0, #8 * REG_SP_EL0]
    msr    tpidrro_el0, x5
    msr    sp_el0, x4

    /* restore the TPIDR0/TPIDR1 */

    ldp    x4,  x5,  [x0, #8 * REG_TPIDR_EL0]
    msr    tpidr_el0, x4
    msr    tpidr_el1, x5

    /* retrieve new thread's SP_ELx */
    ldr    x4, [x0, #8 * REG_SP_ELX]
    mov    sp, x4

#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
    stp    xzr, x30, [sp, #-16]!
    bl     arm64_trace_context_switch
    ldp    xzr, x30, [sp], #16
#endif

#ifdef CONFIG_ARCH_FPU
    stp    xzr, x30, [sp, #-16]!
    bl     arm64_fpu_context_restore
    ldp    xzr, x30, [sp], #16
#endif

    /* Return to arm64_sync_exc() or arm64_irq_handler() */

    ret

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

    mrs    x0, esr_el1
    lsr    x1, x0, #26

#ifdef CONFIG_ARCH_FPU
    /* fpu trap */

    cmp    x1, #0x07 /*Access to SIMD or floating-point */
    bne    1f
    mov    x0, sp
    bl     arm64_fpu_trap

    /* when the fpu trap is handled */

    b      arm64_exit_exc_fpu_done
1:
#endif
    /* 0x15 = SVC system call */

    cmp    x1, #0x15

    /* if this is a svc call ?*/

    bne    exc_handle

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

    /* Save the return value into the */

    str    x0, [sp, #8 * REG_X0]

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
    bl    arm64_syscall_switch

    /* get save task reg context pointer */

    ldr    x1, [sp, #8 * REG_X1]
    cmp    x1, #0x0

    beq do_switch
    ldr x1, [x1]

do_switch:
#ifdef CONFIG_SMP
    /* Notes:
     * Complete any pending TLB or cache maintenance on this CPU in case
     * the thread migrates to a different CPU.
     * This full barrier is also required by the membarrier system
     * call.
     */

    dsb   ish
#endif

    bl    arm64_context_switch

#ifdef CONFIG_ARCH_FPU
    /* when the fpu trap is handled */

    b    arm64_exit_exc_fpu_done
#else
    b    arm64_exit_exception
#endif

save_context:
    arm64_exception_context_save x0 x1 sp

    mov    x0, sp
    bl    arm64_syscall_save_context

    /* Save the return value into the ESF */

    str    x0, [sp, #8 * REG_X0]

    /* Return from exception */

    b    arm64_exit_exception

exc_handle:
    arm64_exception_context_save x0 x1 sp
    mov    x0, #K_ERR_CPU_EXCEPTION
    mov    x1, sp

    /* void arm64_fatal_error(unsigned int reason, const uint64_t *regs)
     * x0 = reason
     * x1 = Exception stack frame
     */

    bl    arm64_fatal_error

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
    /* switch to IRQ stack and save current sp on it. */
#ifdef CONFIG_SMP
    get_cpu_id x1
    ldr    x0, =(g_cpu_int_stacktop)
    lsl    x1, x1, #3
    ldr    x0, [x0, x1]
#else
    ldr    x0, =(g_interrupt_stack + CONFIG_ARCH_INTERRUPTSTACK)
#endif
    /* save the task's stack and switch irq stack */

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

    /* retrieve the task's stack. */

    mov    sp, x1

    cmp    x0, x1
    beq    irq_exit

irq_context_switch:
#ifdef CONFIG_SMP
    /* Notes:
     * Complete any pending TLB or cache maintenance on this CPU in case
     * the thread migrates to a different CPU.
     * This full barrier is also required by the membarrier system
     * call.
     */
    dsb   ish

#endif

    /*  Switch thread
     *  - x0: restore task reg context, return by arm64_decodeirq,
     *  - x1: save task reg context, save before call arm64_decodeirq
     *    call arm64_context_switch(x0) to switch
     */
    bl    arm64_context_switch
#ifdef CONFIG_ARCH_FPU
    /* when the fpu trap is handled */

    b     arm64_exit_exc_fpu_done
#endif

irq_exit:
    b     arm64_exit_exception

/* TODO: if the arm64_fatal_error return success, maybe need context switch */

GTEXT(arm64_serror_handler)
SECTION_FUNC(text, arm64_serror_handler)
    arm64_exception_context_save x0 x1 sp

    mov    x0, #K_ERR_CPU_EXCEPTION
    mov    x1, sp

    bl    arm64_fatal_error
    /* Return here only in case of recoverable error */

    b    arm64_exit_exception

GTEXT(arm64_mode32_error)
SECTION_FUNC(text, arm64_mode32_error)
    arm64_exception_context_save x0 x1 sp

    mov    x1, sp
    mov    x0, #K_ERR_CPU_MODE32

    bl    arm64_fatal_error
    /* Return here only in case of recoverable error */

    b    arm64_exit_exception

GTEXT(arm64_irq_spurious)
SECTION_FUNC(text, arm64_irq_spurious)
    arm64_exception_context_save x0 x1 sp

    mov    x1, sp
    mov    x0, #K_ERR_SPURIOUS_IRQ /* K_ERR_SPURIOUS_IRQ */

    bl    arm64_fatal_error

    /* Return here only in case of recoverable error */

    b    arm64_exit_exception

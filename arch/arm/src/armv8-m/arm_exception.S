/****************************************************************************
 * arch/arm/src/armv8-m/arm_exception.S
 *
 *   Copyright (C) 2009-2013, 2015-2016, 2018 Gregory Nutt.
 *   All rights reserved.
 *   Copyright (C) 2012 Michael Smith. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include <arch/armv8-m/nvicpri.h>

#include "chip.h"
#include "exc_return.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
  /* In protected mode without an interrupt stack, this interrupt handler will set the
   * MSP to the stack pointer of the interrupted thread.  If the interrupted thread
   * was a privileged thread, that will be the MSP otherwise it will be the PSP.  If
   * the PSP is used, then the value of the MSP will be invalid when the interrupt
   * handler returns because it will be a pointer to an old position in the
   * unprivileged stack.  Then when the high priority interrupt occurs and uses this
   * stale MSP, there will most likely be a system failure.
   *
   * If the interrupt stack is selected, on the other hand, then the interrupt
   * handler will always set the MSP to the interrupt stack.  So when the high
   * priority interrupt occurs, it will either use the MSP of the last privileged
   * thread to run or, in the case of the nested interrupt, the interrupt stack if
   * no privileged task has run.
   */

#  if defined(CONFIG_BUILD_PROTECTED) && CONFIG_ARCH_INTERRUPTSTACK < 8
#    error Interrupt stack must be used with high priority interrupts in protected mode
#  endif

  /* Use the BASEPRI to control interrupts is required if nested, high
   * priority interrupts are supported.
   */

#  ifndef CONFIG_ARMV8M_USEBASEPRI
#    error CONFIG_ARMV8M_USEBASEPRI must be used with CONFIG_ARCH_HIPRI_INTERRUPT
#  endif
#endif

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl		exception_common

	.syntax		unified
	.thumb
	.file		"arm_exception.S"

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: setintstack
 *
 * Description:
 *   Set the current stack pointer to the  "top" the interrupt stack.  Single CPU
 *   case.  Must be provided by MCU-specific logic in the SMP case.
 *
 ****************************************************************************/

#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.macro	setintstack, tmp1, tmp2
#ifdef CONFIG_ARMV8M_STACKCHECK_HARDWARE
	ldr		\tmp1, =g_intstackalloc
	msr		msplim, \tmp1
#endif
	ldr		sp, =g_intstacktop
	.endm
#endif

/****************************************************************************
 * .text
 ****************************************************************************/

/* Common exception handling logic.  On entry here, the return stack is on either
 * the PSP or the MSP and looks like the following:
 *
 *      REG_XPSR
 *      REG_R15
 *      REG_R14
 *      REG_R12
 *      REG_R3
 *      REG_R2
 *      REG_R1
 * MSP->REG_R0
 *
 * And
 *      IPSR contains the IRQ number
 *      R14 Contains the EXC_RETURN value
 *      We are in handler mode and the current SP is the MSP
 *
 * If CONFIG_ARCH_FPU is defined, the volatile FP registers and FPSCR are on the
 * return stack immediately above REG_XPSR.
 */

	.text
	.thumb_func
	.type	exception_common, function
exception_common:

	mrs		r0, ipsr				/* R0=exception number */

	/* Complete the context save */

	/* The EXC_RETURN value tells us whether the context is on the MSP or PSP */

	tst		r14, #EXC_RETURN_PROCESS_STACK		/* nonzero if context on process stack */
	beq		1f					/* Branch if context already on the MSP */
	mrs		r1, psp					/* R1=The process stack pointer (PSP) */
	mov		sp, r1					/* Set the MSP to the PSP */
1:
	mov		r2, sp					/* R2=Copy of the main/process stack pointer */
	add		r2, #HW_XCPT_SIZE			/* R2=MSP/PSP before the interrupt was taken */
								/* (ignoring the xPSR[9] alignment bit) */
#ifdef CONFIG_ARMV8M_STACKCHECK_HARDWARE
	mov		r3, #0x0

	ittee		eq
	mrseq		r1, msplim
	msreq		msplim, r3
	mrsne		r1, psplim
	msrne		psplim, r3

	stmdb		sp!, {r1}
#endif

#ifdef CONFIG_ARMV8M_USEBASEPRI
	mrs		r3, basepri				/* R3=Current BASEPRI setting */
#else
	mrs		r3, primask				/* R3=Current PRIMASK setting */
#endif

#ifdef CONFIG_ARCH_FPU

	/* Save the non-volatile FP registers here.
	 *
	 * This routine is the only point where we can save these registers; either before
	 * or after calling arm_doirq.  The compiler is free to use them at any time as long
	 * as they are restored before returning, so we can't assume that we can get at the
	 * true values of these registers in any routine called from here.
	 *
	 * REVISIT: we could do all this saving lazily on the context switch side if we knew
	 * where to put the registers.
	 */

	/* Switched-out task including volatile FP registers ? */

	tst		r14, #EXC_RETURN_STD_CONTEXT
	ite		eq
	vstmdbeq	sp!, {s16-s31}			/* Save the non-volatile FP context */
	subne		sp, #(4*SW_FPU_REGS)
#endif

	stmdb		sp!, {r2-r11,r14}		/* Save the remaining registers plus the SP/PRIMASK values */

	/* There are two arguments to arm_doirq:
	 *
	 *   R0 = The IRQ number
	 *   R1 = The top of the stack points to the saved state
	 */

	mov		r1, sp

#if CONFIG_ARCH_INTERRUPTSTACK > 7
	/* If CONFIG_ARCH_INTERRUPTSTACK is defined, we will set the MSP to use
	 * a special special interrupt stack pointer.  The way that this is done
	 * here prohibits nested interrupts without some additional logic!
	 */

	setintstack	r2, r3			/* SP = IRQ stack top */
#else
	/* Otherwise, we will re-use the interrupted thread's stack.  That may
	 * mean using either MSP or PSP stack for interrupt level processing (in
	 * kernel mode).
	 */

	/* If the interrupt stack is disabled, reserve xcpcontext to ensure
	 * that signal processing can have a separate xcpcontext to handle
	 * signal context (reference: arm_schedulesigaction.c):
	 *      ----------------------
	 *     |    IRQ XCP context   |
	 *      -------------------
	 *     |  Signal XCP context  |
	 *      ----------------------   <- SP
	 * also the sp should be restore after arm_doirq()
	 */

	sub		r2, r1, #XCPTCONTEXT_SIZE		/* Reserve signal context */
	bic		r2, r2, #7				/* Get the stack pointer with 8-byte alignment */
	mov		sp, r2					/* Instantiate the aligned stack */
#endif

	bl		arm_doirq				/* R0=IRQ, R1=register save (msp) */

	/* On return from arm_doirq, R0 will hold a pointer to register context
	 * array to use for the interrupt return.
	 */

	ldmia		r0!, {r2-r11,r14}	/* Recover R4-R11, r14 + 2 temp values */
#ifdef CONFIG_ARCH_FPU
	/* Switched-in task including volatile FP registers ? */

	tst		r14, #EXC_RETURN_STD_CONTEXT
	ite		eq
	vldmiaeq	r0!, {s16-s31}		/* Recover S16-S31 */
	addne		r0, #(4*SW_FPU_REGS)
#endif

#ifdef CONFIG_ARMV8M_STACKCHECK_HARDWARE
	ldmia		r0!, {r1}		/* Get psplim/msplim */
#endif

	/* The EXC_RETURN value tells us whether we are returning on the MSP or PSP
	 */

#ifdef CONFIG_BUILD_PROTECTED
	/* The EXC_RETURN value will be 0xfffffff9 (privileged thread) or 0xfffffff1
	 * (handler mode) if the stack is on the MSP.  It can only be on the PSP if
	 * EXC_RETURN is 0xfffffffd (unprivileged thread)
	 */

	mrs		r2, control				/* R2=Contents of the control register */
	tst		r14, #EXC_RETURN_PROCESS_STACK		/* nonzero if context on process stack */
	beq		2f					/* Branch if privileged */

	orr		r2, r2, #1				/* Unprivileged mode */
#ifdef CONFIG_ARMV8M_STACKCHECK_HARDWARE
	msr		psplim, r1
#endif
	msr		psp, r0					/* R0=The process stack pointer */
	b		3f
2:
	bic		r2, r2, #1				/* Privileged mode */
#ifdef CONFIG_ARMV8M_STACKCHECK_HARDWARE
	msr		msplim, r1
#endif
	msr		msp, r0					/* R0=The main stack pointer */
3:
	msr		control, r2				/* Save the updated control register */
#else
	tst		r14, #EXC_RETURN_PROCESS_STACK		/* nonzero if context on process stack */
#ifdef CONFIG_ARMV8M_STACKCHECK_HARDWARE
	itete		eq
	msreq		msplim, r1
	msrne		psplim, r1
#else
	ite		eq					/* Next two instructions conditional */
#endif
	msreq		msp, r0					/* R0=The main stack pointer */
	msrne		psp, r0					/* R0=The process stack pointer */
#endif

	/* Restore the interrupt state */

#ifdef CONFIG_ARMV8M_USEBASEPRI
	msr		basepri, r3				/* Restore interrupts priority masking */
#else
	msr		primask, r3				/* Restore interrupts */
#endif

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) select the correct stack.
	 */

	bx		r14					/* And return */

	.size	exception_common, .-exception_common

/****************************************************************************
 *  Name: g_intstackalloc/g_intstacktop
 *
 * Description:
 *   Shouldn't happen
 *
 ****************************************************************************/

#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.bss
	.global	g_intstackalloc
	.global	g_intstacktop
	.balign	8
g_intstackalloc:
	.skip	((CONFIG_ARCH_INTERRUPTSTACK + 4) & ~7)
g_intstacktop:
	.size	g_intstackalloc, .-g_intstackalloc
#endif

	.end

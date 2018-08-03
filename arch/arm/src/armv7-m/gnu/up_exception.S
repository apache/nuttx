/************************************************************************************
 * arch/arm/src/armv7-m/gnu/up_exception.S
 *
 *   Copyright (C) 2009-2013, 2015-2016, 2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include "exc_return.h"

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
  /* In kernel mode without an interrupt stack, this interrupt handler will set the
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

#  if defined(CONFIG_BUILD_PROTECTED) && CONFIG_ARCH_INTERRUPTSTACK < 4
#    error Interrupt stack must be used with high priority interrupts in kernel mode
#  endif

  /* Use the BASEPRI to control interrupts is required if nested, high
   * priority interrupts are supported.
   */

#  ifndef CONFIG_ARMV7M_USEBASEPRI
#    error CONFIG_ARMV7M_USEBASEPRI must be used with CONFIG_ARCH_HIPRI_INTERRUPT
#  endif
#endif

/************************************************************************************
 * Public Symbols
 ************************************************************************************/

	.globl		exception_common

	.syntax		unified
	.thumb
	.file		"up_exception.S"

/************************************************************************************
 * Macro Definitions
 ************************************************************************************/

/************************************************************************************
 * Name: setintstack
 *
 * Description:
 *   Set the current stack pointer to the  "top" the interrupt stack.  Single CPU
 *   case.  Must be provided by MCU-specific logic in the SMP case.
 *
 ************************************************************************************/

#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.macro	setintstack, tmp1, tmp2
	ldr		sp, =g_intstackbase
	.endm
#endif

/************************************************************************************
 * .text
 ************************************************************************************/

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
	.type	exception_common, function
	.thumb_func
exception_common:

	mrs		r0, ipsr				/* R0=exception number */

	/* Complete the context save */

	/* The EXC_RETURN value tells us whether the context is on the MSP or PSP */

	tst		r14, #EXC_RETURN_PROCESS_STACK /* nonzero if context on process stack */
	beq		1f						/* Branch if context already on the MSP */
	mrs		r1, psp					/* R1=The process stack pointer (PSP) */
	mov     sp, r1					/* Set the MSP to the PSP */

1:
	mov		r2, sp					/* R2=Copy of the main/process stack pointer */
	add		r2, #HW_XCPT_SIZE		/* R2=MSP/PSP before the interrupt was taken */
									/* (ignoring the xPSR[9] alignment bit) */
#ifdef CONFIG_ARMV7M_USEBASEPRI
	mrs		r3, basepri				/* R3=Current BASEPRI setting */
#else
	mrs		r3, primask				/* R3=Current PRIMASK setting */
#endif

#ifdef CONFIG_ARCH_FPU

	/* Save the non-volatile FP registers here.
	 *
	 * This routine is the only point where we can save these registers; either before
	 * or after calling up_doirq.  The compiler is free to use them at any time as long
	 * as they are restored before returning, so we can't assume that we can get at the
	 * true values of these registers in any routine called from here.
	 *
	 * REVISIT: we could do all this saving lazily on the context switch side if we knew
	 * where to put the registers.
	 */

	vstmdb	sp!, {s16-s31}			/* Save the non-volatile FP context */

#endif

	stmdb	sp!, {r2-r11,r14}		/* Save the remaining registers plus the SP/PRIMASK values */

#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
	/* Disable interrupts, select the stack to use for interrupt handling
	 * and call up_doirq to handle the interrupt
	 */

	cpsid	i						/* Disable further interrupts */

#else
	/* Set the BASEPRI register so that further normal interrupts will be
	 * masked.  Nested, high priority may still occur, however.
	 *
	 * REVISIT:  I think this is un-necessary:  "When the processor is
	 * executing an exception handler, an exception can preempt the exception
	 * handler if its priority is higher than the priority of the exception
	 * being handled. See Interrupt priority grouping for more information
	 * about preemption by an interrupt."
	 */

	mov		r2, #NVIC_SYSH_DISABLE_PRIORITY
	msr		basepri, r2				/* Set the BASEPRI */
#endif

	/* There are two arguments to up_doirq:
	 *
	 *   R0 = The IRQ number
	 *   R1 = The top of the stack points to the saved state
	 */

	mov		r1, sp

	/* Also save the top of the stack in a preserved register */

	mov		r4, sp

#if CONFIG_ARCH_INTERRUPTSTACK > 7
	/* If CONFIG_ARCH_INTERRUPTSTACK is defined, we will set the MSP to use
	 * a special special interrupt stack pointer.  The way that this is done
	 * here prohibits nested interrupts without some additional logic!
	 */

	setintstack	r2, r3

#else
	/* Otherwise, we will re-use the interrupted thread's stack.  That may
	 * mean using either MSP or PSP stack for interrupt level processing (in
	 * kernel mode).
	 */

	bic		r2, r4, #7				/* Get the stack pointer with 8-byte alignment */
	mov		sp, r2					/* Instantiate the aligned stack */

#endif

	bl		up_doirq				/* R0=IRQ, R1=register save (msp) */
	mov		r1, r4					/* Recover R1=main stack pointer */

	/* On return from up_doirq, R0 will hold a pointer to register context
	 * array to use for the interrupt return.  If that return value is the same
	 * as current stack pointer, then things are relatively easy.
	 */

	cmp		r0, r1					/* Context switch? */
	beq		2f						/* Branch if no context switch */

	/* We are returning with a pending context switch.  This case is different
	 * because in this case, the register save structure does not lie on the
	 * stack but, rather within a TCB structure.  We'll have to copy some
	 * values to the stack.
	 */

	/* Copy the hardware-saved context to the stack, and restore the software
	 * saved context directly.
	 *
	 * XXX In the normal case, it appears that this entire operation is unnecessary;
	 *     context switch time would be improved if we could work out when the stack
	 *     is dirty and avoid the work...
	 */

	add		r1, r0, #SW_XCPT_SIZE 	/* R1=Address of HW save area in reg array */
	ldmia	r1!, {r4-r11}			/* Fetch eight registers in HW save area */
#ifdef CONFIG_ARCH_FPU
	vldmia	r1!, {s0-s15}			/* Fetch sixteen FP registers in HW save area */
	ldmia	r1, {r2-r3}				/* Fetch FPSCR and Reserved in HW save area */
#endif
	ldr		r1, [r0, #(4*REG_SP)]	/* R1=Value of SP before interrupt */
#ifdef CONFIG_ARCH_FPU
	stmdb	r1!, {r2-r3}			/* Store FPSCR and Reserved on the return stack */
	vstmdb	r1!, {s0-s15}			/* Store sixteen FP registers on the return stack */
#endif
	stmdb	r1!, {r4-r11}			/* Store eight registers on the return stack */
	ldmia	r0!, {r2-r11,r14}		/* Recover R4-R11, r14 + 2 temp values */
#ifdef CONFIG_ARCH_FPU
	vldmia	r0, {s16-s31}			/* Recover S16-S31 */
#endif

	b		3f						/* Re-join common logic */

2:
	/* We are returning with no context switch.  We simply need to "unwind"
	 * the same stack frame that we created at entry.
	 */

	ldmia	r1!, {r2-r11,r14}		/* Recover R4-R11, r14 + 2 temp values */
#ifdef CONFIG_ARCH_FPU
	vldmia  r1!, {s16-s31}			/* Recover S16-S31 */
#endif

3:
	/* The EXC_RETURN value tells us whether we are returning on the MSP or PSP
	 */

#ifdef CONFIG_BUILD_PROTECTED
	/* The EXC_RETURN value will be 0xfffffff9 (privileged thread) or 0xfffffff1
	 * (handler mode) if the stack is on the MSP.  It can only be on the PSP if
	 * EXC_RETURN is 0xfffffffd (unprivileged thread)
	 */

	mrs		r2, control				/* R2=Contents of the control register */
	tst		r14, #EXC_RETURN_PROCESS_STACK /* nonzero if context on process stack */
	beq		4f						/* Branch if privileged */

	orr		r2, r2, #1				/* Unprivileged mode */
	msr		psp, r1					/* R1=The process stack pointer */
	b		5f
4:
	bic		r2, r2, #1				/* Privileged mode */
	msr		msp, r1					/* R1=The main stack pointer */
5:
	msr		control, r2				/* Save the updated control register */
#else
	tst		r14, #EXC_RETURN_PROCESS_STACK /* nonzero if context on process stack */
	ite		eq						/* next two instructions conditional */
	msreq	msp, r1					/* R1=The main stack pointer */
	msrne	psp, r1					/* R1=The process stack pointer */
#endif

	/* Restore the interrupt state */

#ifdef CONFIG_ARMV7M_USEBASEPRI
	msr		basepri, r3				/* Restore interrupts priority masking */
#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
	cpsie	i						/* Re-enable interrupts */
#endif

#else
	msr		primask, r3				/* Restore interrupts */
#endif

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) select the correct stack.
	 */

	bx		r14						/* And return */

	.size	exception_common, .-exception_common

/************************************************************************************
 *  Name: g_intstackalloc/g_intstackbase
 *
 * Description:
 *   Shouldn't happen
 *
 ************************************************************************************/

#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.bss
	.global	g_intstackalloc
	.global	g_intstackbase
	.align	8
g_intstackalloc:
	.skip	((CONFIG_ARCH_INTERRUPTSTACK + 4) & ~7)
g_intstackbase:
	.size	g_intstackalloc, .-g_intstackalloc
#endif

	.end


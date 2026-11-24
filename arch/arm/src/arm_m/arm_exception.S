/****************************************************************************
 * arch/arm/src/arm_m/arm_exception.S
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2009-2013, 2015-2016, 2018 Gregory Nutt.
 * SPDX-FileCopyrightText: 2012 Michael Smith. All rights reserved.
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
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
#include <arch/arm_m/nvicpri.h>

#include "chip.h"
#include "exc_return.h"
#include "ghs_compat.S"
#include "psr.h"

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
#endif

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl		exception_common
#ifdef __ghs__
	.thumb2
#else
	.syntax		unified
	.thumb
#endif
	.file		"arm_exception.S"

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
	.align 2
	.section    .text.exception_common, "ax"
#ifdef __ghs__
	.type	exception_common, $function
#else
	.thumb_func
	.syntax	unified
	.type	exception_common, function
#endif
exception_common:
	.cfi_sections	.debug_frame
	.cfi_startproc
	mrs		r12, control				/* R12=control */

#ifdef CONFIG_ARMV8M_STACKCHECK_HARDWARE
	mov		r3, #0x0
#endif
	/* Complete the context save */

	/* The EXC_RETURN value tells us whether the context is on the MSP or PSP */

	mov		r0, r14					/* Copy EXC_RETURN */
	lsls		r0, #(31 - EXC_RETURN_PROCESS_BITNO)	/* Move to bit 31 */
	bpl		1f					/* Test bit 31 */
	mrs		r1, psp					/* R1=The process stack pointer (PSP) */
#ifdef CONFIG_ARMV8M_STACKCHECK_HARDWARE
	mrs		r0, psplim
	msr		psplim, r3
#endif
	b		2f
1:
	mrs		r1, msp					/* R1=The main stack pointer (MSP) */
	mov		r0, r1					/* Keep r1 not changed for push */
	subs		r0, #SW_XCPT_SIZE			/* Reserve MSP for SW_XCPT_SIZE */
	msr		msp, r0					/* Reserved stack space */
#ifdef CONFIG_ARMV8M_STACKCHECK_HARDWARE
	mrs		r0, msplim
	msr		msplim, r3
#endif
	isb		sy
2:
	mov		r2, r1					/* R2=Copy of the main/process stack pointer */
	adds		r2, #HW_XCPT_SIZE			/* R2=MSP/PSP before the interrupt was taken */
								/* (ignoring the xPSR[9] alignment bit) */
#ifdef CONFIG_ARMV8M_STACKCHECK_HARDWARE
	stmdb		r1!, {r0, r3}
#endif

#ifdef CONFIG_ARCH_ARMV6M
	mrs		r3, primask				/* R3=Current PRIMASK setting */
#else
	mrs		r3, basepri				/* R3=Current BASEPRI setting */
#endif /* CONFIG_ARCH_ARMV6M */

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
	vstmdbeq	r1!, {s16-s31}			/* Save the non-volatile FP context */
	subne		r1, #(4*SW_FPU_REGS)

#ifdef CONFIG_ARM_HAVE_MVE
	/* the FPSCR[18:16] LTPSIZE field must be set to 0b100 for
	 * "Tail predication not applied" as it's reset value.
	 */

	mov		r0, #ARM_FPSCR_LTPSIZE_NONE
	vmsr		fpscr, r0
#endif
#endif

#ifdef CONFIG_ARCH_ARMV6M
	subs		r1, #(4*SW_INT_REGS)		/* Move cursor to base, for save low regs first */
	mov		r0, r1				/* Copy the context array pointer */
	stmia		r0!, {r2-r7}			/* Save the SP, PRIMASK, and R4-R7 in the context array */

	/* Save R8-R11 control and the EXEC_RETURN value in the context array */

	mov		r2, r8				/* Copy high registers to low */
	mov		r3, r9
	mov		r4, r10
	mov		r5, r11
	mrs		r6, control			/* R6=control */
	mov		r7, r14
	stmia		r0!, {r2-r7}			/* Save the high registers r8-r11 control and r14 */
#else
	stmdb		r1!, {r2-r12,r14}		/* Save the remaining registers plus the SP/PRIMASK values */
#endif /* CONFIG_ARCH_ARMV6M */

	/* There are two arguments to arm_doirq:
	 *
	 *   R0 = The IRQ number
	 *   R1 = The top of the stack points to the saved state
	 */

	mrs		r0, ipsr			/* R0=exception number */

#if CONFIG_ARCH_INTERRUPTSTACK < 7
	/* If CONFIG_ARCH_INTERRUPTSTACK is not defined, we will reuse the
	 * interrupted thread's stack.  That may mean using either MSP or PSP
	 * stack for interrupt level processing (in kernel mode).
	 */

	/* If the interrupt stack is disabled, reserve xcpcontext to ensure
	 * that signal processing can have a separate xcpcontext to handle
	 * signal context (reference: arm_schedulesigaction.c):
	 *      ----------------------
	 *     |    IRQ XCP context   |
	 *      ----------------------
	 *     |  Signal XCP context  |
	 *      ----------------------   <- SP
	 * also the sp should be restore after arm_doirq()
	 */

	mov		r2, r14
	lsls		r2, #(31 - EXC_RETURN_THREAD_BITNO)	/* Move to bit 31 */
	bpl		3f					/* Test bit 31 */
#ifdef CONFIG_ENABLE_ALL_SIGNALS
	mov		r2, r1
	subs		r2, #XCPTCONTEXT_SIZE			/* Reserve signal context */
	movs		r4, #7
	bics		r2, r4					/* Get the stack pointer with 8-byte alignment */
	mov		sp, r2					/* Instantiate the aligned stack */
#endif
3:
#endif
	mov		fp,	r1
	.cfi_def_cfa	fp,	0				/* Register in fp, so we just set fp as frame */
	.cfi_offset	pc,	REG_PC * 4
	.cfi_offset	sp,	REG_SP * 4
	.cfi_offset	lr,	REG_LR * 4
	bl		arm_doirq				/* R0=IRQ, R1=register save (msp) */

	/* On return from arm_doirq, R0 will hold a pointer to register context
	 * array to use for the interrupt return.
	 */

#ifdef CONFIG_ARCH_ARMV6M
	/* Recover R8-R11 and EXEC_RETURN (5 registers) */

	movs		r2, #(4*REG_R8)		/* R2=Offset to R8 storage */
	adds		r1, r0, r2		/* R1=Address of R8 storage */
	ldmia		r1!, {r2-r7}		/* Recover R8-R11 control and R14 (6 registers)*/
	mov		r8, r2			/* Move to position in high registers */
	mov		r9, r3
	mov		r10, r4
	mov		r11, r5
	msr		control, r6
	mov		r14, r7			/* EXEC_RETURN */

	/* Recover SP (R2), PRIMASK (R3), and R4-R7. Determine the value of
	 * the stack pointer as it was on entry to the exception handler.
	 */

	movs		r1, #(4*REG_R8)		/* Already restored offset */
	ldmia		r0!, {r2-r7}		/* Recover R4-R7 + 2 temp values */
	adds		r0, r0, r1		/* Fix already restored R8-R11 and R14 popped address */
#else
	ldmia		r0!, {r2-r12,r14}	/* Recover R4-R12, r14 + 2 temp values */
#endif /* CONFIG_ARCH_ARMV6M */
#ifdef CONFIG_ARCH_FPU
	/* Switched-in task including volatile FP registers ? */

	tst		r14, #EXC_RETURN_STD_CONTEXT
	ite		eq
	vldmiaeq	r0!, {s16-s31}		/* Recover S16-S31 */
	addne		r0, #(4*SW_FPU_REGS)
#endif

#ifdef CONFIG_ARMV8M_STACKCHECK_HARDWARE
	ldmia		r0!, {r1}		/* Get psplim/msplim */
	add		r0, r0, #4
#endif

	/* The EXC_RETURN value tells us whether we are returning on the MSP or PSP
	 */

	mov		r2, r14					/* Copy EXC_RETURN */
	lsls		r2, #(31 - EXC_RETURN_PROCESS_BITNO)	/* Move to bit 31 */
	bmi		4f					/* Test bit 31 */
	msr		msp, r0					/* R1=The main stack pointer */
#ifdef CONFIG_ARMV8M_STACKCHECK_HARDWARE
	msr		msplim, r1
#endif
	b		5f
4:
	msr		psp, r0					/* R1=The process stack pointer */
#ifdef CONFIG_ARMV8M_STACKCHECK_HARDWARE
	msr		psplim, r1
#endif
5:
	isb		sy

	/* Restore the interrupt state */

#ifdef CONFIG_ARCH_ARMV6M
	msr		primask, r3				/* Restore interrupts priority masking*/
#else
	msr		basepri, r3				/* Restore interrupts priority masking */
	msr		control, r12				/* Restore control */
#endif /* CONFIG_ARCH_ARMV6M */

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) select the correct stack.
	 */

	bx		r14					/* And return */
	.cfi_endproc
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

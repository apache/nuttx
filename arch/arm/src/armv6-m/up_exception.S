/************************************************************************************
 * arch/arm/src/armv6-m/up_exception.S
 *
 *   Copyright (C) 2009-2013 Gregory Nutt. All rights reserved.
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
 * Global Symbols
 ************************************************************************************/

	.globl		exception_common

	.cpu		cortex-m0
	.file		"up_exception.S"

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
 */

	.text
	.align	2
	.code	16
	.thumb_func
	.type	exception_common, function
exception_common:

	mrs		r0, ipsr				/* R0=exception number */

	/* Complete the context save */

	/* The EXC_RETURN value tells us whether the context is on the MSP or PSP */

	mov		r0, r14					/* Copy high register to low register */
	lsl		r1, r0, #(31 - EXC_RETURN_PROCESS_BITNO)	/* Move to bit 31 */
	bmi		1f						/* Test bit 31 */
	mrs		r1, msp					/* R1=The main stack pointer */
	b		2f

1:
	mrs		r1, psp					/* R1=The process stack pointer */

2:
	mov		r2, #XCPTCONTEXT_SIZE	/* R2=Size of context array */
	sub		r0, r1, r2				/* R0=Beginning of context array on the stack */
	mov		r2, r1					/* R2=Copy of the main/process stack pointer */
	add		r2, #HW_XCPT_SIZE		/* R2=MSP/PSP before the interrupt was taken */
									/* (ignoring the xPSR[9] alignment bit) */
	mrs		r3, basepri				/* R3=Current BASEPRI setting */

	stmia	r0!, {r2-r7}			/* Save the SP, BASEPRI, and R4-R7 */
	mov		r2, r8					/* Copy high registers to low */
	mov		r3, r9
	mov		r4, r10
	mov		r5, r11
	mov		r6, r14
	stmia	r0!, {r2-r6}			/* Save the high registers r8-r11 and r14 */

	/* Disable interrupts, select the stack to use for interrupt handling
	 * and call up_doirq to handle the interrupt
	 */

	cpsid	i						/* Disable further interrupts */

	/* If CONFIG_ARCH_INTERRUPTSTACK is defined, we will use a special interrupt
	 * stack pointer.  The way that this is done here prohibits nested interrupts!
	 * Otherwise, we will use the stack that was current when the interrupt was taken.
	 */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	ldr		sp, =g_intstackbase
	push	{r1}					/* Save the MSP on the interrupt stack */
	bl		up_doirq				/* R0=IRQ, R1=register save area on stack */
	pop		{r1}					/* Recover R1=main stack pointer */
#else
	msr	 msp, r1					/* We are using the main stack pointer */
	bl		up_doirq				/* R0=IRQ, R1=register save area on stack */
	mrs	 r1, msp					/* Recover R1=main stack pointer */
#endif

	/* On return from up_doirq, r0 will hold a pointer to register context
	 * array to use for the interrupt return.  If that return value is the same
	 * as current stack pointer, then things are relatively easy.
	 */

	cmp		r0, r1					/* Context switch? */
	beq		3f						/* Branch if no context switch */

	/* We are returning with a pending context switch.  This case is different
	 * because in this case, the register save structure does not lie on the
	 * stack but, rather within a TCB structure.  We'll have to copy some
	 * values to the stack.
	 */

	/* Copy the hardware-saved context to the new stack */

	mov		r2, #SW_XCPT_SIZE		/* R2=Size of sofware-saved portion of the context array */
	add		r1, r0, r2				/* R1=Address of HW save area in reg array */
	ldr		r2, [r0, #(4*REG_SP)]	/* R1=Value of SP before interrupt */
	sub		r2, #HW_XCPT_SIZE		/* R1=Address of HW save area on stack */
	ldmia	r1!, {r4-r7}			/* Fetch four registers from the HW save area */
	stmia	r2!, {r4-r7}			/* Copy four registers to the return stack */
	ldmia	r1!, {r4-r7}			/* Fetch four registers from the HW save area */
	stmia	r2!, {r4-r7}			/* Copy four registers to the return stack */

	/* Restore the register contents */

	mov		r1, r0

3:
	/* We are returning with no context switch.  We simply need to "unwind"
	 * the same stack frame that we created at entry.
	 */

	mov		r2, #(4*REG_R8)			/* R2=Offset to R8 storage */
	sub		r0, r1, r2				/* R0=Address of R8 storage */
	ldmia	r0!, {r2-r6}			/* Recover R8-R11 and R14 */
	mov		r8, r2					/* Move to position in high registers */
	mov		r9, r3
	mov		r10, r4
	mov		r11, r5
	mov		r14, r6
	ldmia	r1!, {r2-r7}			/* Recover R4-R7 + 2 temp values */

	/* The EXC_RETURN value tells us whether the context is on the MSP or PSP */

	mov		r0, r14					/* Copy high register to low register */
	lsl		r1, r0, #(31 - EXC_RETURN_PROCESS_BITNO)	/* Move to bit 31 */
	bmi		5f						/* Test bit 31 */
	mrs		r1, msp					/* R1=The main stack pointer */
	b		6f

5:
	mrs		r1, psp					/* R1=The process stack pointer */

6:
	/* Restore the interrupt state */

	msr		basepri, r3				/* Restore interrupts priority masking*/
	cpsie	i						/* Re-enable interrupts */

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) select the correct stack.
	 */

	bx		r14						/* And return */

	.size	exception_common, .-exception_common

/************************************************************************************
 *  Name: up_interruptstack/g_intstackbase
 *
 * Description:
 *   Shouldn't happen
 *
 ************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.bss
	.global	g_intstackbase
	.align	4
up_interruptstack:
	.skip	(CONFIG_ARCH_INTERRUPTSTACK & ~3)
g_intstackbase:
	.size	up_interruptstack, .-up_interruptstack
#endif

	.end


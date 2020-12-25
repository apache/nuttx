/************************************************************************************************
 * arch/arm/src/armv7-m/gnu/up_lazyexcption.S
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
 ************************************************************************************************/

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>
#include <arch/armv7-m/nvicpri.h>

#include "chip.h"
#include "exc_return.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* Configuration ********************************************************************************/

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
  /* In kernel mode without an interrupt stack, this interrupt handler will set the MSP to the
   * stack pointer of the interrupted thread.  If the interrupted thread was a privileged
   * thread, that will be the MSP otherwise it will be the PSP.  If the PSP is used, then the
   * value of the MSP will be invalid when the interrupt handler returns because it will be a
   * pointer to an old position in the unprivileged stack.  Then when the high priority
   * interrupt occurs and uses this stale MSP, there will most likely be a system failure.
   *
   * If the interrupt stack is selected, on the other hand, then the interrupt handler will
   * always set the MSP to the interrupt stack.  So when the high priority interrupt occurs,
   * it will either use the MSP of the last privileged thread to run or, in the case of the
   * nested interrupt, the interrupt stack if no privileged task has run.
   */

#  if defined(CONFIG_BUILD_PROTECTED) && CONFIG_ARCH_INTERRUPTSTACK < 8
#    error Interrupt stack must be used with high priority interrupts in kernel mode
#  endif

  /* Use the BASEPRI to control interrupts is required if nested, high
   * priority interrupts are supported.
   */

#  ifndef CONFIG_ARMV7M_USEBASEPRI
#    error CONFIG_ARMV7M_USEBASEPRI must be used with CONFIG_ARCH_HIPRI_INTERRUPT
#  endif
#endif

/************************************************************************************************
 * Public Symbols
 ************************************************************************************************/

	.globl		exception_common

	.syntax		unified
	.thumb
	.file		"arm_lazyexception.S"

/************************************************************************************************
 * Macro Definitions
 ************************************************************************************************/

/************************************************************************************************
 * Name: setintstack
 *
 * Description:
 *   Set the current stack pointer to the  "top" the interrupt stack.  Single CPU case.  Must be
 *   provided by MCU-specific logic in chip.h for the SMP case.
 *
 ************************************************************************************************/

#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.macro	setintstack, tmp1, tmp2
	ldr		sp, =g_intstackbase
	.endm
#endif

/************************************************************************************************
 * .text
 ************************************************************************************************/

/* Common IRQ handling logic.  On entry here, the return stack is on either
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
	.type		exception_common, function

exception_common:

	/* Get the IRQ number from the IPSR */

	mrs			r0, ipsr			/* R0=exception number */

	/* Complete the context save */

#ifdef CONFIG_BUILD_PROTECTED
	/* The EXC_RETURN value will be 0xfffffff9 (privileged thread) or 0xfffffff1
	 * (handler mode) if the stack is on the MSP.  It can only be on the PSP if
	 * EXC_RETURN is 0xfffffffd (unprivileged thread)
	 */

	tst		r14, #EXC_RETURN_PROCESS_STACK /* nonzero if context on process stack */
	beq		1f						/* Branch if context already on the MSP */
	mrs		r1, psp					/* R1=The process stack pointer (PSP) */
	mov     sp, r1					/* Set the MSP to the PSP */

1:
#endif

	/* r1 holds the value of the stack pointer AFTER the exception handling logic
	 * pushed the various registers onto the stack.  Get r2 = the value of the
	 * stack pointer BEFORE the interrupt modified it.
	 */

	mov		r2, sp					/* R2=Copy of the main/process stack pointer */
	add		r2, #HW_XCPT_SIZE		/* R2=MSP/PSP before the interrupt was taken */

#ifdef CONFIG_ARMV7M_USEBASEPRI
	mrs		r3, basepri				/* R3=Current BASEPRI setting */
#else
	mrs		r3, primask				/* R3=Current PRIMASK setting */
#endif

#ifdef CONFIG_ARCH_FPU
	/* Skip over the block of memory reserved for floating pointer register save.
	 * Lazy FPU register saving is used.  FPU registers will be saved in this
	 * block only if a context switch occurs (this means, of course, that the FPU
	 * cannot be used in interrupt processing).
	 */

	sub		sp, #(4*SW_FPU_REGS)
#endif

	/* Save the remaining registers on the stack after the registers pushed
	 * by the exception handling logic. r2=SP and r3=primask or basepri, r4-r11,
	 * r14=register values.
	 */

#ifdef CONFIG_BUILD_PROTECTED
	stmdb	sp!, {r2-r11,r14}		/* Save the remaining registers plus the SP value */
#else
	stmdb	sp!, {r2-r11}			/* Save the remaining registers plus the SP value */
#endif

	/* There are two arguments to arm_doirq:
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

	setintstack	r2, r3				/* SP = IRQ stack top */

#else
	/* Otherwise, we will re-use the interrupted thread's stack.  That may
	 * mean using either MSP or PSP stack for interrupt level processing (in
	 * kernel mode).
	 */

	bic		r2, r4, #7				/* Get the stack pointer with 8-byte alignment */
	mov		sp, r2					/* Instantiate the aligned stack */

#endif

	bl		arm_doirq				/* R0=IRQ, R1=register save (msp) */
	mov		r1, r4					/* Recover R1=main stack pointer */

	/* On return from arm_doirq, R0 will hold a pointer to register context
	 * array to use for the interrupt return.  If that return value is the same
	 * as current stack pointer, then things are relatively easy.
	 */

	cmp		r0, r1					/* Context switch? */
	beq		2f						/* Branch if no context switch */

	/* We are returning with a pending context switch.
	 *
	 * If the FPU is enabled, then we will need to restore FPU registers.
	 * This is not done in normal interrupt save/restore because the cost
	 * is prohibitive.  This is only done when switching contexts.  A
	 * consequence of this is that floating point operations may not be
	 * performed in interrupt handling logic.
	 *
	 * Here:
	 *   r0 = Address of the register save area
	 *
	 * NOTE: It is a requirement that arm_restorefpu() preserve the value of
	 * r0!
	 */

#ifdef CONFIG_ARCH_FPU
	bl		arm_restorefpu			/* Restore the FPU registers */
#endif

	/* We are returning with a pending context switch.  This case is different
	 * because in this case, the register save structure does not lie in the
	 * stack but, rather, within a TCB structure.  We'll have to copy some
	 * values to the stack.
	 */

	add		r1, r0, #SW_XCPT_SIZE	/* R1=Address of HW save area in reg array */
	ldmia	r1, {r4-r11}			/* Fetch eight registers in HW save area */
	ldr		r1, [r0, #(4*REG_SP)]	/* R1=Value of SP before interrupt */
	stmdb	r1!, {r4-r11}			/* Store eight registers in HW save area */
#ifdef CONFIG_BUILD_PROTECTED
	ldmia	r0, {r2-r11,r14}		/* Recover R4-R11, r14 + 2 temp values */
#else
	ldmia	r0, {r2-r11}			/* Recover R4-R11 + 2 temp values */
#endif
	b		3f						/* Re-join common logic */

	/* We are returning with no context switch.  We simply need to "unwind"
	 * the same stack frame that we created
	 *
	 * Here:
	 *   r1 = Address of the return stack (same as r0)
	 */

2:
#ifdef CONFIG_BUILD_PROTECTED
	ldmia	r1!, {r2-r11,r14}		/* Recover R4-R11, r14 + 2 temp values */
#else
	ldmia	r1!, {r2-r11}			/* Recover R4-R11 + 2 temp values */
#endif

#ifdef CONFIG_ARCH_FPU
	/* Skip over the block of memory reserved for floating pointer register
	 * save. Then R1 is the address of the HW save area
	 */

	add		r1, #(4*SW_FPU_REGS)
#endif

	/* Set up to return from the exception
	 *
	 * Here:
	 *   r1 = Address on the target thread's stack position at the start of
	 *        the registers saved by hardware
	 *   r3 = primask or basepri
	 *   r4-r11 = restored register values
	 */

3:

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
	msr		msp, r1					/* Recover the return MSP value */

	/* Preload r14 with the special return value first (so that the return
	 * actually occurs with interrupts still disabled).
	 */

	ldr		r14, =EXC_RETURN_PRIVTHR	/* Load the special value */
#endif

	/* Restore the interrupt state */

#ifdef CONFIG_ARMV7M_USEBASEPRI
	msr		basepri, r3				/* Restore interrupts priority masking */
#else
	msr		primask, r3				/* Restore interrupts */
#endif

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) continue to use the MSP
	 */

	bx		r14						/* And return */
	.size	exception_common, .-exception_common

/************************************************************************************************
 *  Name: g_intstackalloc/g_intstackbase
 *
 * Description:
 *   Shouldn't happen
 *
 ************************************************************************************************/

#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.bss
	.global	g_intstackalloc
	.global	g_intstackbase
	.balign	8
g_intstackalloc:
	.skip	((CONFIG_ARCH_INTERRUPTSTACK + 4) & ~7)
g_intstackbase:
	.size	g_intstackalloc, .-g_intstackalloc
#endif

	.end

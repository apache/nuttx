/************************************************************************************
 * arch/arm/src/tiva/tiva_vectors.S
 *
 *   Copyright (C) 2009-2010, 2013-2014 Gregory Nutt. All rights reserved.
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

#include "chip.h"
#include "exc_return.h"

/************************************************************************************
 * Configuration
 ************************************************************************************/

/************************************************************************************
 * Preprocessor Definitions
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
   * handler will always set the the MSP to the interrupt stack.  So when the high
   * priority interrupt occurs, it will either use the MSP of the last privileged
   * thread to run or, in the case of the nested interrupt, the interrupt stack if
   * no privileged task has run.
   */

#  if defined(CONFIG_NUTTX_KERNEL) && CONFIG_ARCH_INTERRUPTSTACK < 4
#    error Interrupt stack must be used with high priority interrupts in kernel mode
#  endif

  /* Use the the BASEPRI to control interrupts is required if nested, high
   * priority interrupts are supported.
   */

#  ifndef CONFIG_ARMV7M_USEBASEPRI
#    error CONFIG_ARMV7M_USEBASEPRI must be used with CONFIG_ARCH_HIPRI_INTERRUPT
#  endif
#endif

/* Memory Map ***********************************************************************/
/*
 * 0x0000:0000 - Beginning of FLASH. Address of vectors (if not using bootloader)
 * 0x0002:0000 - Address of vectors if using bootloader
 * 0x0003:ffff - End of flash
 * 0x2000:0000 - Start of SRAM and start of .data (_sdata)
 *             - End of .data (_edata) abd start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE = end of idle stack, start of heap
 * 0x2000:ffff - End of SRAM and end of heap
 */

#define IDLE_STACK		(_ebss+CONFIG_IDLETHREAD_STACKSIZE-4)
#define HEAP_BASE		(_ebss+CONFIG_IDLETHREAD_STACKSIZE)

/************************************************************************************
 * Global Symbols
 ************************************************************************************/

	.syntax		unified
	.thumb
	.file		"tiva_vectors.S"

/* Check if common ARMv7 interrupt vectoring is used (see arch/arm/src/armv7-m/up_vectors.S) */

#ifndef CONFIG_ARMV7M_CMNVECTOR

	.globl		__start

/************************************************************************************
 * Macros
 ************************************************************************************/

/* On entry into an IRQ, the hardware automatically saves the xPSR, PC, LR, R12, R0-R3
 * registers on the stack, then branches to an instantantiation of the following
 * macro.  This macro simply loads the IRQ number into R0, then jumps to the common
 * IRQ handling logic.
 */

	.macro	HANDLER, label, irqno
	.thumb_func
\label:
	mov		r0, #\irqno
	b		exception_common
	.endm

/************************************************************************************
 * Vectors
 ************************************************************************************/

	.section	.vectors, "ax"
	.code		16
	.align		2
	.globl		_vectors
	.type		_vectors, function

_vectors:

/* Processor Exceptions */

	.word	IDLE_STACK			/* Vector  0: Reset stack pointer */
	.word	__start				/* Vector  1: Reset vector */
	.word	tiva_nmi			/* Vector  2: Non-Maskable Interrupt (NMI) */
	.word	tiva_hardfault		/* Vector  3: Hard fault */
	.word	tiva_mpu			/* Vector  4: Memory management (MPU) */
	.word	tiva_busfault		/* Vector  5: Bus fault */
	.word	tiva_usagefault		/* Vector  6: Usage fault */
	.word	tiva_reserved		/* Vector  7: Reserved */
	.word	tiva_reserved		/* Vector  8: Reserved */
	.word	tiva_reserved		/* Vector  9: Reserved */
	.word	tiva_reserved		/* Vector 10: Reserved */
	.word	tiva_svcall			/* Vector 11: SVC call */
	.word	tiva_dbgmonitor		/* Vector 12: Debug monitor */
	.word	tiva_reserved		/* Vector 13: Reserved */
	.word	tiva_pendsv			/* Vector 14: Pendable system service request */
	.word	tiva_systick		/* Vector 15: System tick */

/* External Interrupts */

/* External Interrupts */

#undef VECTOR
#define VECTOR(l,i) .word l

#undef UNUSED
#define UNUSED(i) .word tiva_reserved

#include "chip/chip/tiva_vectors.h"
	.size	_vectors, .-_vectors

/************************************************************************************
 * .text
 ************************************************************************************/

	.text
	.type	handlers, function
	.thumb_func
handlers:
	HANDLER	tiva_reserved, TIVA_IRQ_RESERVED		/* Unexpected/reserved vector */
	HANDLER	tiva_nmi, TIVA_IRQ_NMI					/* Vector  2: Non-Maskable Interrupt (NMI) */
	HANDLER	tiva_hardfault, TIVA_IRQ_HARDFAULT		/* Vector  3: Hard fault */
	HANDLER	tiva_mpu, TIVA_IRQ_MEMFAULT				/* Vector  4: Memory management (MPU) */
	HANDLER	tiva_busfault, TIVA_IRQ_BUSFAULT		/* Vector  5: Bus fault */
	HANDLER	tiva_usagefault, TIVA_IRQ_USAGEFAULT	/* Vector  6: Usage fault */
	HANDLER	tiva_svcall, TIVA_IRQ_SVCALL			/* Vector 11: SVC call */
	HANDLER	tiva_dbgmonitor, TIVA_IRQ_DBGMONITOR	/* Vector 12: Debug Monitor */
	HANDLER	tiva_pendsv, TIVA_IRQ_PENDSV			/* Vector 14: Penable system service request */
	HANDLER	tiva_systick, TIVA_IRQ_SYSTICK			/* Vector 15: System tick */

#undef VECTOR
#define VECTOR(l,i) HANDLER l, i

#undef UNUSED
#define UNUSED(i)

#include "chip/chip/tiva_vectors.h"

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
 *      R0 contains the IRQ number
 *      R14 Contains the EXC_RETURN value
 *      We are in handler mode and the current SP is the MSP
 */

	.globl		exception_common
	.type		exception_common, function
exception_common:

	/* Complete the context save */

#ifdef CONFIG_NUTTX_KERNEL
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

	/* r1 holds the value of the stack pointer AFTER the excption handling logic
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

#ifdef CONFIG_NUTTX_KERNEL
	stmdb	sp!, {r2-r11,r14}		/* Save the remaining registers plus the SP value */
#else
	stmdb	sp!, {r2-r11}			/* Save the remaining registers plus the SP value */
#endif

#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
	/* Disable interrupts, select the stack to use for interrupt handling
	 * and call up_doirq to handle the interrupt
	 */

	cpsid	i						/* Disable further interrupts */

#else
	/* Set the BASEPRI register so that further normal interrupts will be
	 * masked.  Nested, high priority may still occur, however.
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

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	/* If CONFIG_ARCH_INTERRUPTSTACK is defined, we will set the MSP to use
	 * a special special interrupt stack pointer.  The way that this is done
	 * here prohibits nested interrupts without some additional logic!
	 */

	ldr		sp, =g_intstackbase
	str		r1, [sp, #-4]!			/* Save the MSP on the interrupt stack */
	bl		up_doirq				/* R0=IRQ, R1=register save (msp) */
	ldr		r1, [sp, #+4]!			/* Recover R1=main stack pointer */

#else
	/* Otherwise, we will re-use the interrupted thread's stack.  That may
	 * mean using either MSP or PSP stack for interrupt level processing (in
	 * kernel mode).
	 */

	bl		up_doirq				/* R0=IRQ, R1=register save (msp) */
	mov		r1, sp					/* Recover R1=main stack pointer */
#endif

	/* On return from up_doirq, R0 will hold a pointer to register context
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
	
	 * NOTE: It is a requirement that up_restorefpu() preserve the value of
	 * r0!
	 */

#ifdef CONFIG_ARCH_FPU
	bl		up_restorefpu			/* Restore the FPU registers */
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
#ifdef CONFIG_NUTTX_KERNEL
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
#ifdef CONFIG_NUTTX_KERNEL
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

#ifdef CONFIG_NUTTX_KERNEL
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
#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
	cpsie	i						/* Re-enable interrupts */
#endif

#else
	msr		primask, r3				/* Restore interrupts */
#endif

	/* Always return with R14 containing the special value that will: (1)
	 * return to thread mode, and (2) continue to use the MSP
	 */

	bx		r14						/* And return */
	.size	handlers, .-handlers

/************************************************************************************
 *  Name: g_intstackalloc/g_intstackbase
 *
 * Description:
 *   Shouldn't happen
 *
 ************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.bss
	.global	g_intstackalloc
	.global	g_intstackbase
	.align	4
g_intstackalloc:
	.skip	(CONFIG_ARCH_INTERRUPTSTACK & ~3)
g_intstackbase:
	.size	g_intstackalloc, .-g_intstackalloc
#endif
#endif /* CONFIG_ARMV7M_CMNVECTOR */

/************************************************************************************
 * .rodata
 ************************************************************************************/

	.section	.rodata, "a"

/* Variables: _sbss is the start of the BSS region (see ld.script) _ebss is the end
 * of the BSS regsion (see ld.script). The idle task stack starts at the end of BSS
 * and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE thread is the thread that
 * the system boots on and, eventually, becomes the idle, do nothing task that runs
 * only when there is nothing else to run.  The heap continues from there until the
 * end of memory.  See g_idle_topstack below.
 */

	.globl	g_idle_topstack
	.type	g_idle_topstack, object
g_idle_topstack:
	.word	HEAP_BASE
	.size	g_idle_topstack, .-g_idle_topstack

	.end

/****************************************************************************
 * arch/arm/src/armv7-a/arm_fullcontextrestore.S
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#include <nuttx/irq.h>
#include "up_internal.h"
#include "svcall.h"

	.file	"arm_fullcontextrestore.S"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	up_fullcontextrestore

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fullcontextrestore
 *
 * Description:
 *   Restore the specified task context. Full prototype is:
 *
 *     void up_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
 *
 * Return:
 *   None
 *
 ****************************************************************************/

	.globl	up_fullcontextrestore
	.type	up_fullcontextrestore, function

up_fullcontextrestore:

	/* On entry, a1 (r0) holds address of the register save area.  All other
	 * registers are available for use.
	 */

#ifdef CONFIG_ARCH_FPU

	/* First, restore the floating point registers.  Lets do this before we
	 * restore the ARM registers so that we have plenty of registers to
	 * work with.
	 */

	add		r1, r0, #(4*REG_S0)		/* r1=Address of FP register storage */

	/* Load all floating point registers.  Registers are loaded in numeric order,
	 * s0, s1, ... in increasing address order.
	 */

	vldmia	r1!, {s0-s31}			/* Restore the full FP context */

	/* Load the floating point control and status register.   At the end of the
	 * vstmia, r1 will point to the FPCSR storage location.
	 */

	ldr		r2, [r1], #4			/* Fetch the floating point control and status register */
	vmsr	fpscr, r2				/* Restore the FPCSR */

#endif

#ifdef CONFIG_BUILD_KERNEL

	/* For the kernel build, we need to be able to transition gracefully
	 * between kernel- and user-mode tasks.  Here we do that with a system
	 * call; the system call will execute in kernel mode and but can return
	 * to either user or kernel mode.
	 */

	/* Perform the System call with R0=SYS_context_restore, R1=restoreregs */

	mov		r1, r0					/* R1: restoreregs */
	mov		r0, #SYS_context_restore	/* R0: SYS_context_restore syscall */
	svc		#0x900001				/* Perform the system call */

	/* This call should not return */

	bx		lr						/* Unnecessary ... will not return */

#else

	/* For a flat build, we can do all of this here... Just think of this as
	 * a longjmp() all on steriods.
	 */

#ifdef CONFIG_SMP

	/* Recover all registers except for the volatile registers {r0-r3, r12}
	 * and r14 (lr).
	 */

	add		r1, r0, #(4*REG_R4)		/* Offset to REG_R2 storage */
	ldmia	r1, {r4-r11}			/* Recover registers */

	/* Recover the stack pointer (r13) */

	ldr		sp, [r0, #(4*REG_SP)]	/* Recover the stack pointer */

	/* Create a stack from to preserve the structure pointer and some
	 * additional registers.  We should have everything preserved on the
	 * in registers on on the stack when irq_restore_lock(0) is called (I am
	 * not sure that is necessary, but I have concerns about the save
	 * structure getting modified in the TCB if the spinlock is released --
	 * assuming that it is set???).
	 */

	sub		sp, sp, #(4*8)			/* Frame for eight registers */

	ldr		r1, [r0, #(4*REG_R2)]	/* Fetch the stored r2 value */
	str		r1, [sp, #(4*0)]		/* Save it in the stack */
	ldr		r1, [r0, #(4*REG_R3)]	/* Fetch the stored r3 value */
	str		r1, [sp, #(4*1)]		/* Save it in the stack */
	ldr		r1, [r0, #(4*REG_R12)]	/* Fetch the stored r12 value */
	str		r1, [sp, #(4*2)]		/* Save it in the stack */
	ldr		r1, [r0, #(4*REG_R14)]	/* Fetch the stored r14 value */
	str		r1, [sp, #(4*3)]		/* Save it in the stack */
	ldr		r1, [r0, #(4*REG_CPSR)]	/* Fetch the stored CPSR value */
	str		r1, [sp, #(4*4)]		/* Save it in the stack */

	ldr		r1, [r0, #(4*REG_R0)]	/* Fetch the stored r0 value */
	str		r1, [sp, #(4*5)]		/* Save it in the stack */
	ldr		r1, [r0, #(4*REG_R1)]	/* Fetch the stored r1 value */
	str		r1, [sp, #(4*6)]		/* Save it in the stack */
	ldr		r1, [r0, #(4*REG_PC)]	/* Fetch the stored pc value */
	str		r1, [sp, #(4*7)]		/* Save it at the bottom of the frame */

	/* In the SMP configuration, critical section management uses a
	 * "voting" algorithm with current task on each CPU casting its
	 * "vote" by the state of the TCB irqcount flag.  That irqcount
	 * for the current task on this CPU will be different is a
	 * context switch occurrred.
	 */

	bl		irq_restore_lock

	/* Recover the structure pointer and most of the volatile structures
	 * that were saved on the stack.
	 */

	ldr		r2,  [sp, #(4*0)]		/* Recover R2 */
	ldr		r3,  [sp, #(4*1)]		/* Recover R3 */
	ldr		r12, [sp, #(4*2)]		/* Recover R12 */
	ldr		r14, [sp, #(4*3)]		/* Recover R14 */
	ldr		r1,  [sp, #(4*4)]		/* Recover the save CPSR in r1 */
	add		sp,   sp, #(4*5)		/* Discard 5 of the allocated 8 storage locations */

#else

	/* Recover all registers except for r0, r1, R15, and CPSR */

	add		r1, r0, #(4*REG_R2)		/* Offset to REG_R2 storage */
	ldmia	r1, {r2-r14}			/* Recover registers */

	/* Create a stack frame to hold the some registers */

	sub		sp, sp, #(3*4)			/* Frame for three registers */
	ldr		r1, [r0, #(4*REG_R0)]	/* Fetch the stored r0 value */
	str		r1, [sp, #(4*0)]		/* Save it at the top of the stack */
	ldr		r1, [r0, #(4*REG_R1)]	/* Fetch the stored r1 value */
	str		r1, [sp, #(4*1)]		/* Save it in the stack */
	ldr		r1, [r0, #(4*REG_PC)]	/* Fetch the stored pc value */
	str		r1, [sp, #(4*2)]		/* Save it at the bottom of the frame */

	/* Recover the saved CPSR value in r1 */

	ldr		r1, [r0, #(4*REG_CPSR)]	/* Fetch the stored CPSR value */
#endif

	/* Now we can restore the CPSR.  We wait until we are completely
	 * finished with the context save data to do this. Restore the CPSR
	 * may re-enable and interrupts and we could be in a context
	 * where the save structure is only protected by interrupts being
	 * disabled.
	 */

	msr		cpsr, r1				/* Set the CPSR */

	/* Now recover r0 and r1 */

	ldr		r0, [sp]
	ldr		r1, [sp, #4]
	add		sp, sp, #(4*2)

	/* Then return to the address at the stop of the stack,
	 * destroying the stack frame
	 */

	ldr		pc, [sp], #4

#endif

	.size up_fullcontextrestore, . - up_fullcontextrestore

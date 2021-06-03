/****************************************************************************
 * arch/arm/src/armv7-a/arm_fullcontextrestore.S
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
#include <arch/irq.h>

#include "svcall.h"
#include "arm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file	"arm_fullcontextrestore.S"

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_fullcontextrestore
 *
 * Description:
 *   Restore the current thread context.  Full prototype is:
 *
 *   void arm_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.globl	arm_fullcontextrestore
	.type	arm_fullcontextrestore, function
arm_fullcontextrestore:

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

#ifdef CONFIG_ARM_HAVE_FPU_D32
	vldmia.64	r1!, {d0-d15}			/* Restore the full FP context */
	vldmia.64	r1!, {d16-d31}
#else
	vldmia		r1!, {s0-s31}			/* Restore the full FP context */
#endif

	/* Load the floating point control and status register.   At the end of the
	 * vstmia, r1 will point to the FPCSR storage location.
	 */

	ldr		r2, [r1], #4			/* Fetch the floating point control and status register */
	vmsr		fpscr, r2			/* Restore the FPCSR */
#endif

#ifdef CONFIG_BUILD_KERNEL
	/* For the kernel build, we need to be able to transition gracefully
	 * between kernel- and user-mode tasks.  Here we do that with a system
	 * call; the system call will execute in kernel mode and but can return
	 * to either user or kernel mode.
	 */

	/* Perform the System call with R0=1 and R1=regs */

	mov		r1, r0				/* R1: regs */
	mov		r0, #SYS_restore_context	/* R0: restore context */
	svc		#SYS_syscall			/* Force synchronous SVCall (or Hard Fault) */

	/* This call should not return */

	bx		lr				/* Unnecessary ... will not return */

#else
	/* For a flat build, we can do all of this here... Just think of this as
	 * a longjmp() all on steroids.
	 */

	/* Recover all registers except for r0, r1, r2, R15, and CPSR */

	add		r1, r0, #(4*REG_R3)		/* Offset to REG_R3 storage */
	ldmia		r1, {r3-r14}			/* Recover registers */

	ldr		r2, [r0, #(4*REG_CPSR)]		/* Fetch the stored CPSR value */

	/* Create a stack frame to hold the some registers */

	sub		sp, sp, #(4*4)			/* Frame for four registers */
	ldr		r1, [r0, #(4*REG_R0)]		/* Fetch the stored r0 value */
	str		r1, [sp]			/* Save it at the top of the stack */
	ldr		r1, [r0, #(4*REG_R1)]		/* Fetch the stored r1 value */
	str		r1, [sp, #4]			/* Save it in the stack */
	ldr		r1, [r0, #(4*REG_R2)]		/* Fetch the stored r2 value */
	str		r1, [sp, #8]			/* Save it in the stack */
	ldr		r1, [r0, #(4*REG_PC)]		/* Fetch the stored pc value */
	tst		r2, #PSR_T_BIT
	orrne		r1, r1, #1
	str		r1, [sp, #12]			/* Save it at the bottom of the frame */

	/* Now we can restore the CPSR.  We wait until we are completely
	 * finished with the context save data to do this. Restore the CPSR
	 * may re-enable and interrupts and we could be in a context
	 * where the save structure is only protected by interrupts being
	 * disabled.
	 */

	msr		spsr, r2			/* Set the SPSR */

	/* Now recover r0-r2, pc and cpsr, destroying the stack frame */

	ldmia		sp!, {r0-r2, pc}^
#endif

	.size	arm_fullcontextrestore, .-arm_fullcontextrestore
	.end

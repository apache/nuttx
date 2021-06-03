/****************************************************************************
 * arch/arm/src/arm/arm_fullcontextrestore.S
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

	/* Recover all registers except for r0, r1, R15, and CPSR */

	add	r1, r0, #(4*REG_R2)	/* Offset to REG_R2 storage */
	ldmia	r1, {r2-r14}		/* Recover registers */

	/* Create a stack frame to hold the some registers */

	sub	sp, sp, #(3*4)		/* Frame for three registers */
	ldr	r1, [r0, #(4*REG_R0)]	/* Fetch the stored r0 value */
	str	r1, [sp]		/* Save it at the top of the stack */
	ldr	r1, [r0, #(4*REG_R1)]	/* Fetch the stored r1 value */
	str	r1, [sp, #4]		/* Save it in the stack */
	ldr	r1, [r0, #(4*REG_PC)]	/* Fetch the stored pc value */
	str	r1, [sp, #8]		/* Save it at the bottom of the frame */

	/* Now we can restore the CPSR.  We wait until we are completely
	 * finished with the context save data to do this. Restore the CPSR
	 * may re-enable and interrupts and we could be in a context
	 * where the save structure is only protected by interrupts being
	 * disabled.
	 */

	ldr	r1, [r0, #(4*REG_CPSR)]	/* Fetch the stored CPSR value */
	msr	spsr, r1		/* Set the SPSR */

	/* Now recover r0-r1, pc and cpsr, destroying the stack frame */

	ldmia	sp!, {r0-r1, r15}^

	.size	arm_fullcontextrestore, .-arm_fullcontextrestore
	.end

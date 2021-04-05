/****************************************************************************
 * arch/arm/src/arm/arm_saveusercontext.S
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

	.file	"arm_saveusercontext.S"

/****************************************************************************
 * Macros
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_saveusercontext
 *
 * Description:
 *   Save the current thread context.  Full prototype is:
 *
 *   int  arm_saveusercontext(uint32_t *saveregs);
 *
 * Returned Value:
 *   0: Normal return
 *   1: Context switch return
 *
 ****************************************************************************/

	.globl	arm_saveusercontext
	.type	arm_saveusercontext, function
arm_saveusercontext:

	/* On entry, a1 (r0) holds address of struct xcptcontext */

	/* Make sure that the return value will be non-zero (the value of the
	 * other volatile registers don't matter -- r1-r3, ip).  This function
	 * is called through the normal C calling conventions and the values of
	 * these registers cannot be assumed at the point of setjmp return.
	 */

	mov	ip, #1
	str	ip, [r0, #(4*REG_R0)]

	/* Save the volatile registers (plus r12 which really
	 * doesn't need to be saved)
	 */

	add	r1, r0, #(4*REG_R4)
	stmia	r1, {r4-r14}

	/* Save the current cpsr */

	mrs	r2, cpsr		/* R2 = CPSR value */
	add	r1, r0, #(4*REG_CPSR)
	str	r2, [r1]

	/* Save the return address as the PC so that we return to the exit from
	 * this function.
	 */

	add	r1, r0, #(4*REG_PC)
	str	lr, [r1]

	/* Return 0 now indicating that this return is not a context switch */

	mov	r0, #0		/* Return value == 0 */
	bx	lr		/* Return */
	.size	arm_saveusercontext, .-arm_saveusercontext
	.end

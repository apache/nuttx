/****************************************************************************
 * arch/arm/src/armv7-a/arm_saveusercontext.S
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

	mov		ip, #1
	str		ip, [r0, #(4*REG_R0)]

	/* Save the volatile registers (plus r12 which really
	 * doesn't need to be saved)
	 */

	add		r1, r0, #(4*REG_R4)
	stmia		r1, {r4-r14}

	/* Save the current cpsr */

	mrs		r2, cpsr		/* R2 = CPSR value */
	add		r1, r0, #(4*REG_CPSR)
	str		r2, [r1]

	/* Save the return address as the PC so that we return to the exit from
	 * this function.
	 */

	ldr		r2, =1f
	add		r1, r0, #(4*REG_PC)
	str		r2, [r1]

	/* Save the floating point registers.
	 * REVISIT:  Not all of the floating point registers need to be saved.
	 * Some are volatile and need not be preserved across functions calls.
	 * But right now, I can't find the definitive list of the volatile
	 * floating point registers.
	 */

#ifdef CONFIG_ARCH_FPU
	add		r1, r0, #(4*REG_S0)	/* R1=Address of FP register storage */

	/* Store all floating point registers.  Registers are stored in numeric order,
	 * s0, s1, ... in increasing address order.
	 */

#ifdef CONFIG_ARM_HAVE_FPU_D32
	vstmia.64	r1!, {d0-d15}		/* Save the full FP context */
	vstmia.64	r1!, {d16-d31}
#else
	vstmia		r1!, {s0-s31}		/* Save the full FP context */
#endif

	/* Store the floating point control and status register.  At the end of the
	 * vstmia, r1 will point to the FPSCR storage location.
	 */

	vmrs		r2, fpscr		/* Fetch the FPSCR */
	str		r2, [r1], #4		/* Save the floating point control and status register */
#endif

	/* Return 0 now indicating that this return is not a context switch */

	mov		r0, #0			/* Return value == 0 */
	bx		lr			/* Return */

1:

	/* Return 1 now indicating that this return is a context switch */

	mov		r0, #1			/* Return value == 1 */
	bx		lr			/* Return */

	.size	arm_saveusercontext, .-arm_saveusercontext
	.end

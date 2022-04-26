/****************************************************************************
 * libs/libc/machine/arm/gnu/arm_setjmp.S
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

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	setjmp
	.globl	longjmp

	.syntax	unified
	.file	"setjmp.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setjmp
 *
 * Description:
 *   Given the pointer to a register save area (in R0), save the state of the
 *   all callee-saved registers
 *
 * C Function Prototype:
 *   int setjmp(jmp_buf env);
 *
 * Input Parameters:
 *   env - A pointer to the register save area in which to save the floating point
 *     registers and core registers.  Since setjmp() can not be inlined, we
 *     only need to save the ABI-specified callee-saved registers.
 *
 * Returned Value:
 *   0      setjmp called directly
 *   non-0  we justed returned from a longjmp()
 *
 ****************************************************************************/

	.type	setjmp, function
setjmp:

	/* Store callee-saved Core registers */

	mov		ip, sp				/* Move sp to ip so we can save it */

#ifdef CONFIG_ARCH_ARMV6M
	stmia		r0!, {r4-r7}			/* Save R4 ~ R7 */

	mov		r2, r8
	mov		r3, r9
	mov		r4, r10
	mov		r5, r11
	mov		r6, ip
	mov		r7, lr
	stmia		r0!, {r2-r7}			/* Save R8 ~ R11, IP, LR */
#else
	stmia		r0!, {r4-r11, ip, lr}
#endif

#ifdef CONFIG_ARCH_FPU
	vstmia		r0!, {s16-s31}			/* Save the callee-saved FP registers */

	/* Store the floating point control and status register.  At the end of the
	 * vstmia, r0 will point to the FPSCR storage location.
	 */

	vmrs		r1, fpscr			/* Fetch the FPSCR */
	str		r1, [r0], #4			/* Save the floating point control and status register */
							/* DSA: don't need to inc r0 */
#endif /* CONFIG_ARCH_FPU */

	/* we're done, we're out of here */

	movs		r0, #0
	bx		lr

	.size	setjmp, .-setjmp

/****************************************************************************
 * Name: longjmp
 *
 * Description:
 *   The longjmp() function used the information saved in env to transfer control
 *   control back to the point where setjmp() was called and to restore ("rewind")
 *   the stack to its state at the time of the setjmp() call.  When control is
 *   passed back to where setjmp() had been called, setjmp() will return with
 *   'val', the second parameter passed to longjmp().
 *
 * C Function Prototype:
 *   void longjmp(jmp_buf env, int val);
 *
 * Input Parameters:
 *   jmp_buf env
 *   int     val
 *
 * Returned Value:
 *   This function does not return anything explicitly.
 *
 ****************************************************************************/

	.type	longjmp, function
longjmp:

	/* Load callee-saved Core registers */

#ifdef CONFIG_ARCH_ARMV6M
	ldmia		r0!, {r4-r7}			/* Restore R4 ~ R7 */

	ldmia		r0!, {r2-r3}			/* Restore R8, R9 */
	mov		r8, r2
	mov		r9, r3

	ldmia		r0!, {r2-r3}			/* Restore R10, R11 */
	mov		r10, r2
	mov		r11, r3

	ldmia		r0!, {r2-r3}			/* Restore IP, LR */
	mov		ip, r2
	mov		lr, r3
#else
	ldmia		r0!, {r4-r11, ip, lr}
#endif

	mov		sp, ip				/* Restore sp */

#ifdef CONFIG_ARCH_FPU
	/* Load callee-saved floating point registers.  */

	vldmia		r0!, {s16-s31}			/* Restore FP context */

	/* Load the floating point control and status register. */

	ldr		r2, [r0], #4			/* Fetch the floating point control and status register */
							/* DSA: don't need to inc r0 */
	vmsr		fpscr, r2			/* Restore the FPSCR */
#endif /* CONFIG_ARCH_FPU */

	mov		r0, r1				/* return val */
	bx		lr

	.size	longjmp, .-longjmp
	.end

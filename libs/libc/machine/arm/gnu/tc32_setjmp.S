/****************************************************************************
 * libs/libc/machine/arm/gnu/tc32_setjmp.S
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
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	setjmp
	.globl	longjmp

	.file	"tc32_setjmp.S"

/****************************************************************************
 * Macros
 ****************************************************************************/

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

	.type		setjmp, function
setjmp:

	/* Store callee-saved Core registers */

	tstorer		r4, [r0, #0]			/* Save R4 */
	tstorer		r5, [r0, #4]			/* Save R5 */
	tstorer		r6, [r0, #8]			/* Save R6 */
	tstorer		r7, [r0, #12]			/* Save R7 */

	tmov		r2, r8				/* Save R8 */
	tmov		r3, r9				/* Save R9 */
	tmov		r4, r10				/* Save R10 */
	tmov		r5, r11				/* Save R11 */
	tmov		r6, sp				/* Save SP/R13 */
	tmov		r7, lr				/* Save LR/R14 */

	tstorer		r2, [r0, #16]			/* Save R8 */
	tstorer		r3, [r0, #20]			/* Save R9 */
	tstorer		r4, [r0, #24]			/* Save R10 */
	tstorer		r5, [r0, #28]			/* Save R11 */
	tstorer		r6, [r0, #32]			/* Save SP/R13 */
	tstorer		r7, [r0, #36]			/* Save LR/R14 */

	tmovs		r0, #0
	tjex		lr

	.size		setjmp, .-setjmp

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

	tloadr		r2, [r0, #16]			/* Restore R8 */
	tloadr		r3, [r0, #20]			/* Restore R9 */
	tloadr		r4, [r0, #24]			/* Restore R10 */
	tloadr		r5, [r0, #28]			/* Restore R11 */
	tloadr		r6, [r0, #32]			/* Restore SP/R13 */
	tloadr		r7, [r0, #36]			/* Restore LR/R14 */

	tmov		r8, r2				/* Restore R8 */
	tmov		r9, r3				/* Restore R9 */
	tmov		r10, r4				/* Restore R10 */
	tmov		r11, r5				/* Restore R11 */
	tmov		sp, r6				/* Restore SP/R13 */
	tmov		lr, r7				/* Restore LR/R14 */

	tloadr		r4, [r0, #0]			/* Restore R4 */
	tloadr		r5, [r0, #4]			/* Restore R5 */
	tloadr		r6, [r0, #8]			/* Restore R6 */
	tloadr		r7, [r0, #12]			/* Restore R7 */

	tmov		r0, r1				/* return val */
	tjex		lr

	.size		longjmp, .-longjmp
	.end

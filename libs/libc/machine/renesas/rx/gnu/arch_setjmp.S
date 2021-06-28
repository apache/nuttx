/****************************************************************************
 * libs/libc/machine/renesas/rx/gnu/arm_setjmp.S
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

	.text
	.global	_setjmp
	.global	_longjmp

	.file	"setjmp.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setjmp
 *
 * Description:
 *   Given the pointer to a register save area (in R1), save the state of the
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

	.type	_setjmp, function
_setjmp:
	/* save Stack Pointer */
	mov.l		r0, [r1]

	/* Store callee-saved registers R6-R13 */
	mov.l		r6, 0x4[r1]
        mov.l   	r7, 0x8[r1]
        mov.l   	r8, 0xc[r1]
        mov.l   	r9, 0x10[r1]
        mov.l   	r10, 0x14[r1]
        mov.l   	r11, 0x18[r1]
        mov.l   	r12, 0x1c[r1]
        mov.l   	r13, 0x20[r1]

	/* get return address from stack */
        mov.l   	[r0], r2

	/* save return address */
        mov.l   	r2, 0x24[r1]
	
	/* return 0 */
        mov     	#0, r1
	rts

	.size	_setjmp, .-_setjmp

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

	.type	_longjmp, function
_longjmp:
	/* check if r2 is zero */
	tst     	r2, r2
	/* set r2 to 1 if it was zero */
        stz     	#1, r2

	/* restore stack pointer */
        mov.l   	[r1], r0

	/* get the saved return address */
        mov.l   	0x24[r1], r3
	/* set our return address */
        mov.l   	r3, [r0]

	/* restore callee-saved registers R6-R13 */
        mov.l   	0x20[r1], r13
        mov.l   	0x1c[r1], r12
        mov.l   	0x18[r1], r11
        mov.l   	0x14[r1], r10
        mov.l   	0x10[r1], r9
        mov.l   	0xc[r1], r8
        mov.l   	0x8[r1], r7
        mov.l   	0x4[r1], r6

	/* return val */
	mov.l		r2, r1
	/* return */
        rts

	.size	_longjmp, .-_longjmp
	.end

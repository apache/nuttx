/*************************************************************************
 * arch/z16/src/z16f/z16f_saveusercontext.asm
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
 *************************************************************************/

/*************************************************************************
 * Included Files
 *************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>
#include "chip.h"

/*************************************************************************
 * External References / External Definitions
 *************************************************************************/

	xdef	_up_saveusercontext

/*************************************************************************
 * Data Allocation
 *************************************************************************/

	define CODESEG, SPACE=EROM
	segment CODESEG

/*************************************************************************
 * Code
 *************************************************************************/

/*************************************************************************
 * Name: up_saveusercontext
 *
 * Description:
 *   Save the current user context.
 *   r0-r7: These are caller saved registers and do not need to be stored
 *	    here
 *   r8-r13: There callee saved registers must be preserved
 *   r14:    Frame pointer
 *   r15:    Stack pointer (with return address on stack)
 *
 * Input Parameters:
 *   r1: pointer to the register save array in the XCPT structure
 *
 *************************************************************************/

_up_saveusercontext:
	/* Save the flags (needed to restore the interrupt state) */

	ld.ub	r3, Z16F_CNTRL_FLAGS	/* Fetch the flags register (zero padded) */
	ld.w	2*REG_FLAGS(r1), r3	/* Save 16-bit value */

	/* Save r8-R13 */

	ld	2*REG_R8(r1), r8	/* Save r8 */
	ld	2*REG_R9(r1), r9	/* Save r9 */
	ld	2*REG_R10(r1), r10	/* Save r10 */
	ld	2*REG_R11(r1), r11	/* Save r11 */
	ld	2*REG_R12(r1), r12	/* Save r12 */
	ld	2*REG_R13(r1), r13	/* Save r13 */

	/* Save the stack pointer and the frame pointer */

	ld	2*REG_FP(r1), fp	/* Save the frame pointer */
	ld	r0, #4			/* 4 bytes of return address on stack */
	add	r0, sp			/* Value of stack pointer on return */
	ld	2*REG_SP(r1), r0	/* Save the stack pointer value on return */

	/* Save the return address at the top of the stack */

	ld	r0, (sp)		/* Save the return address */
	ld	2*REG_PC(r1), r0

	/* Set the return value so that if when the task is restarted
	 * (via z16f_restoreusercontext() or via interrupt handling return),
	 * the returned value will be 1
	 */

	ld	r0, #1
	ld	2*REG_R0(r1), r0

	/* But always return 0 when returning from this function.  The
	 * apparent return value tells the higher level logic whether the
	 * user context was saved or restored (in the spirit of setjmp and longjmp)
	 */

	clr	r0			/* Always returns 0 */
	ret
	end

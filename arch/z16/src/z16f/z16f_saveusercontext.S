/*************************************************************************
 * arch/z16/src/z16f/z16f_saveusercontext.asm
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSeqUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *************************************************************************/

/*************************************************************************
 * Included Files
 *************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>
#include "chip/chip.h"

/*************************************************************************
 * External References / External Definitions
 *************************************************************************/

	xdef	up_saveusercontext

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
 * Parameters:
 *   r1: pointer to the register save array in the XCPT structure
 *
 *************************************************************************/

up_saveusercontext:
	/* Save the flags (needed to restore the interrupt state) */
	
	ld	r3, Z16F_CNTRL_FLAGS	/* Fetch the flags register */
	sll	r3, #8			/* Pad with zero */
	ld	2*REG_FLAGS(r1), r3

	/* Save r8-R13 */

	ld	2*REG_R8(r1), r8	/* Save r8 */
	ld	2*REG_R8(r1), r9	/* Save r9 */
	ld	2*REG_R8(r1), r10	/* Save r10 */
	ld	2*REG_R8(r1), r11	/* Save r11 */
	ld	2*REG_R8(r1), r12	/* Save r12 */
	ld	2*REG_R8(r1), r13	/* Save r13 */

	/* Save the stack pointer and the frame pointer */

	ld	2*REG_FP(r1), fp	/* Save the frame pointer */
	ld	2*REG_SP(r1), sp	/* Save the stack pointer */

	/* Save the return address at the top of the stack */

	ld	r0, (sp)		/* Save the return address */
	ld	2*REG_PC(r1), r0	
	clr	r0			/* Always returns 0 */
	ret
	end

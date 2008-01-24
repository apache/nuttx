/*************************************************************************
 * arch/z16/src/z16f/z16f_restoreusercontext.asm
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

	xdef	_up_restoreusercontext

/*************************************************************************
 * Data Allocation
 *************************************************************************/

	define CODESEG, SPACE=EROM
	segment CODESEG

/*************************************************************************
 * Code
 *************************************************************************/

/*************************************************************************
 * Name: up_restoreusercontext
 *
 * Description:
 *   Restore the current user context.
 *
 * Parameters:
 *   r1: pointer to the register save array in the XCPT structure
 *
 *************************************************************************/

_up_restoreusercontext:
	/* Disable interrupts throughout the following */

	di

	/* Set up the stack for return */

	ld	sp, 2*REG_SP(r1)			/* sp=Value of SP on return */
	ld	r2, 2*REG_PC(r1)			/* r2=return address */
	push	r1					/* Push the return address onto the stack */

	/* Retore r8-r18 */

	ld	sp, r1					/* sp=Value of SP at call to _up_doirq */
	popmhi	<r8-r14>				/* Restore r8-r14 */

	/* Test if interrupts should be enabled upon return */

	ld.w	r2, 2*REG_FLAGS(r1)			/* r2=padded flags value */
	tm	r2, #(Z16F_CNTRL_FLAGS_IRQE << 8)	/* Test (shifted) IRQE bit */
	jp	z, _up_returndisabled			/* Jump if interrupts should remain disabled */

	/* Return with interrupts enabled */

	popmlo	<r0-r7>					/* Restore r0-r7 */
	ei						/* Enable interrupts and return */
	ret

	/* Return with interrupts disabled */

_up_returndisabled:
	popmlo	<r0-r7>					/* Restore r0-r7 */
	ret
	end

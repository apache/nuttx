/**************************************************************************
 * up_restorecontext.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/irq.h>
#include "up_internal.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_restorecontext
 *
 * Description:
 *   Restore the stack specified in the context structure and return to
 *   that context
 *
 * Inputs:
 *   context - Holds the stack content of the context to return to
 *
 * Return:
 *   This function does not return.
 *
 **************************************************************************/

void up_restorecontext(FAR struct xcptcontext *context) __naked
{
  _asm
	ar2 = 0x02
	ar3 = 0x03
	ar4 = 0x04
	ar5 = 0x05
	ar6 = 0x06
	ar7 = 0x07
	ar0 = 0x00
	ar1 = 0x01

#ifdef CONFIG_SWITCH_FRAME_DUMP
	push	dpl
	push	dph
	lcall	_up_dumpframe
	pop	dph
	pop	dpl
#endif

	/* Interrupts should be disabled for the following.  up_popcontext() will
	 * set the new interrupt state correctly.
	 */

	clr	ea

	/* Register usage in the following:
	 *
	 * R0   - Holds working the 8-bit IRAM pointer
	 * R1   - Not used
	 * R2-3 - Holds the working 16-bit XRAM pointer
	 * R4   - Holds the working byte count
	 * R5   - Holds the new stack pointer
	 * R6-7 - Not used
	 */

	/* Fetch r4 = context->nbytes */

	movx	a, @dptr
	mov	r4, a

	/* Save the new stack pointer in r5 */

	add	a, #(STACK_BASE-1)
	mov	r5, a

	/* Save r2-3 = &context->stack */

	inc	dptr
	mov	r2, dpl
	mov	r3, dph

	/* Set r0 = stack base address */

	mov	r0, #STACK_BASE

	/* Top of the copy loop */
00001$:
	dec	r4
	jz	00002$

	/* Fetch the next byte from context->stack */

	mov	dpl, r2
	mov	dph, r3
	movx	a,@dptr

	/* Increment the XRAM pointer */

	inc	dptr
	mov	r2, dpl
	mov	r3, dph

	/* Save the next byte into IRAM */

	mov	@r0, a

	/* Increment the IRAM pointer */

	inc	r0
	sjmp	00001$
00002$:

	/* Set the new stack pointer */

	mov	sp, r5

#ifdef CONFIG_SWITCH_FRAME_DUMP
	lcall	_up_dumpstack
#endif
	/* Then restore the context from the stack */

	pop	_bp 
	pop	psw
	pop	ar1 
	pop	ar0 
	pop	ar7 
	pop	ar6 
	pop	ar5 
	pop	ar4 
	pop	ar3 
	pop	ar2 
	pop	b 
	pop	dph
	pop	dpl 

	/* Restore the interrupt state per the stored IE value */

	pop	acc
	jb	acc.7,00003$
	clr	ie.7
	sjmp	00004$
  00003$:
	setb	ie.7
  00004$:

	pop acc
	ret
  _endasm;
}



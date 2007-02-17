/**************************************************************************
 * up_saveusercontext.S
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
 * Name: up_saveusercontext
 **************************************************************************/

	.text
	.globl	up_saveusercontext
	.type	up_saveusercontext, function
up_saveusercontext:
	/* On entry, a1 (r0) holds address of struct xcptcontext.
	 * Offset to the user region.
	 */

	/* Make sure that the return value will be non-zero (the
	 * value of the other volatile registers don't matter --
	 * r1-r3, ip).  This function is called throught the
	 * noraml C calling conventions and the values of these
	 * registers cannot be assumed at the point of setjmp
	 * return.
	 */

        mov	ip, #1
	str	ip, [r0, #JB_R0]

	/* Get the offset to the user save area */

	add	r0, r0, #XCPTCONTEXT_UOFFSET

	/* Get the current cpsr as well */

	mrs	r3, cpsr	/* R3 = CPSR value */

	/* We need to save:
	 *
	 * Volatile register:	r3 (holds the cpsr value)
	 * Static registers:	v1-v7 (aka r4-r10)
	 * Frame pointer:	fp (aka r11)
	 * Stack pointer:	sp (aka r13)
	 * Return address:	lr (aka r14)
	 *
	 * These have to be save in the same order as is done
	 * by the interrupt handling logic.
	 */

	stmia	r0, {r3-r11, r13-r14}

	/* Return 0 */

	mov	r0, #0		/* Return value == 0 */
	mov	pc, lr		/* Return */
	.size	up_saveusercontext, . - up_saveusercontext


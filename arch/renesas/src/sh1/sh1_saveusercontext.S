/**************************************************************************
 * arch/renesas/src/sh1/sh1_saveusercontext.S
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/irq.h>
#include "up_internal.h"

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Public Data
 **************************************************************************/

/**************************************************************************
 * Private Data
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
	.globl	_up_saveusercontext
	.type	_up_saveusercontext, function
_up_saveusercontext:

	/* On entry, R4 contains a reference to the register save structure
	 * to populate.
	 *
	 * Re-position to the end of the structure (+4_ so that we can use
	 * auto decrement
	 */

	add	#(XCPTCONTEXT_SIZE), r4	/* R4: Address of last entry + 4 */

	/* Save the necessary registers */

	stc     sr, r0			/* Get and save the SR */
	mov.l	r0, @-r4
	sts.l	pr, @-r4		/* The return address is the restore PC */
	add	#-(7*4), r4		/* Skip over r4, r7-r1.  R4 points at R1 storage */

	/* Save a nonzero value for R0 in the register save area.  This will
	 * will be the apparent return value when a context switch back to
	 * this thread is performed.
	 */

	mov	#1, r0			/* Save R0=1 (return value) */
	mov.l	r0, @-r4
	add	#-(2*4), r4		/* Skip over mach and macl.  R4 points at R15 storage. */
	mov.l	r15, @-r4		/* SP */
	stc.l	gbr, @-r4		/* GBR & PR */
	sts.l	pr, @-r4

	mov.l	r14, @-r4		/* R14-R8 */
	mov.l	r13, @-r4
	mov.l	r12, @-r4
	mov.l	r11, @-r4
	mov.l	r10, @-r4
	mov.l	r9, @-r4
	mov.l	r8, @-r4

	/* And return zero */

	rts
	mov	#0, r0
	.size	up_saveusercontext, . - up_saveusercontext
	.end

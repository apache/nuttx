/*************************************************************************
 * arch/z16/src/z16f/z16f_restoreusercontext.asm
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

	xdef	_z16_restoreusercontext

/*************************************************************************
 * Data Allocation
 *************************************************************************/

	define CODESEG, SPACE=EROM
	segment CODESEG

/*************************************************************************
 * Code
 *************************************************************************/

/*************************************************************************
 * Name: z16_restoreusercontext
 *
 * Description:
 *   Restore the current user context.
 *
 * Input Parameters:
 *   r1: pointer to the register save array in the XCPT structure
 *
 *************************************************************************/

_z16_restoreusercontext:
	/* Disable interrupts while we are accessing the TCB's register
	 * save structure.  The correct interrupt state will be restored
	 * by the iret instruction below.
	 */

	di

	/* Set up the stack for return from 'interrupt' */

	ld	sp, 2*REG_SP(r1)			/* sp=Value of SP on return */
	ld.w	r2, 2*REG_FLAGS(r1)			/* r1=padded flags value */
	push.w	r2					/* Push padded flags value onto the stack */
	ld	r2, 2*REG_PC(r1)			/* r2=return address */
	push	r2					/* Push the return address onto the stack */
	ld	r7, 2*REG_R7(r1)			/* r7=saved value of r7 */
	push	r7					/* Save this so that we can use r7 */
	ld	r7, sp					/* r7=saved SP value */

	/* Restore registers and return from interrupt */

	ld	sp, r1					/* sp=Pointer to register save structure */
	popmhi	<r8-r14>				/* Restore r8-r14 */
	add	sp, #4					/* Skip over restore of r15=sp */
	popmlo	<r0-r6>					/* Restore r0-r6 */
	ld	sp, r7					/* Switch to the correct stack for return */
	pop	r7					/* Recover the saved value of r7 from the stack */
	iret2						/* Return from interrupt */

	end

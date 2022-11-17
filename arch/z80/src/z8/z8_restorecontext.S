/**************************************************************************
 * arch/z80/src/z8/z8_saveusercontext.S
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

#include <nuttx/config.h>
#include <arch/irq.h>
#include <ez8.inc>

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

	xdef	_z8_restorecontext

/**************************************************************************
 * Code
 **************************************************************************/

	segment	CODE

/****************************************************************************
 * Name: _z8_restorecontext
 *
 * Description:
 *   Restore the task context that was previously saved via
 *   _up_saveusercontext() or by interrupt handling.  Unlike the
 *   _up_saveusercontext() counterpart, we do not know the context of the
 *   restored task and, hence, we must handle the worst case -- restore
 *   everythihng.
 *
 * Input Parameters:
 *   On entry, the following stack organization is assumed:
 *
 *          Pointer to the context save structure
 *   TOS -> Return address (2)
 *
 * Assumptions:
 *   Large model, dynamic frames
 *
 **************************************************************************/

_z8_restorecontext:
	/* Disable all interrupts because we are going to be using
	 * the IRQ register set.
	 */

	di

	/* Switch to IRQ register set */

	srp     #%f0

	/* Get the rr0 = the current value of the stack pointer */

	ldx	r0, sph			/* rr0 = stack pointer */
	ldx	r1, spl

	/* Get rr6 = the pointer to the context save structure */

	ldx	r6, 2(rr0)		/* rr6 = pointer to context structure */
	ldx	r7, 3(rr0)

	/* Copy all registers into the user register area.  NOTE: we
	 * use the saved RP value to determine the destination address.
	 */

	clr	r0			/* rr0 = destination address */
	ldx	r1, XCPT_RP_OFFS(rr6)
	ld	r2, r6			/* rr2 = source address */
	ld	r3, r7
	ld	r4, #16			/* r4 = number of bytes to copy */

_z8_restore:
	ldx	r5, @rr2
	ldx	@rr0, r5
	incw	rr0
	incw	rr2
	djnz	r4, _z8_restore

	/* Set the new stack pointer */

	ldx	r0, XCPT_SPH_OFFS(rr6)
	ldx	r1, XCPT_SPL_OFFS(rr6)
	ldx	sph, r0
	ldx	spl, r1

	/* Push the return address onto the stack */

	ldx	r0, XCPT_PCH_OFFS(rr6)
	ldx	r1, XCPT_PCL_OFFS(rr6)
	push	r1
	push	r0

	/* Recover the flags and RP settings.. but don't restore them yet */

	ldx	r1, XCPT_FLAGS_OFFS(rr6)
	ldx	r2, XCPT_RP_OFFS(rr6)

	/* Determine whether interrupts must be enabled on return.  This
	 * would be nicer to do below, but later we will need to preserve
	 * the condition codes in the flags.
	 */

	ldx	r0, XCPT_IRQCTL_OFFS(rr6)
	tm	r0, #%80
	jr	nz, _z8_returnenabled

	/* Restore the flag settings */

	ldx	flags, r1

	/* Restore the user register page and return with interrupts disabled */

	ldx	rp, r2				/* Does not effect flags */
	ret					/* Does not effect flags */

_z8_returnenabled:
	/* Restore the flag settings */

	ldx	flags, r1

	/* Restore the user register page, re-enable interrupts and return */

	ldx	rp, r2				/* Does not effect flags */
	ei					/* Does not effect flags */
	ret					/* Does not effect flags */

	end

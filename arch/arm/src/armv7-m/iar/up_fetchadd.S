/****************************************************************************
 * arch/arm/src/armv7-m/iar/up_fetchadd.S
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

	MODULE	up_testset
	SECTION	.text:CODE:NOROOT(2)

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	PUBLIC	up_fetchadd32
	PUBLIC	up_fetchsub32
	PUBLIC	up_fetchadd16
	PUBLIC	up_fetchsub16
	PUBLIC	up_fetchadd8
	PUBLIC	up_fetchsub8

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	THUMB

/****************************************************************************
 * Name: up_fetchadd32
 *
 * Description:
 *   Perform an atomic fetch add operation on the provided 32-bit value.
 *
 *   This function must be provided via the architecture-specific logic.
 *
 * Input Parameters:
 *   addr  - The address of 32-bit value to be incremented.
 *   value - The 32-bit addend
 *
 * Returned Value:
 *   The incremented value (volatile!)
 *
 ****************************************************************************/

up_fetchadd32:

	ldrex	r2, [r0]			/* Fetch the value to be incremented */
	add		r2, r2, r1			/* Add the addend */

	strex	r3, r2, [r0]		/* Attempt to save the result */
	teq		r3, #0				/* r3 will be 1 if strex failed */
	bne		up_fetchadd32		/* Failed to lock... try again */

	mov		r0, r2				/* Return the incremented value */
	bx		lr					/* Successful! */

/****************************************************************************
 * Name: up_fetchsub32
 *
 * Description:
 *   Perform an atomic fetch subtract operation on the provided 32-bit value.
 *
 *   This function must be provided via the architecture-specific logic.
 *
 * Input Parameters:
 *   addr  - The address of 32-bit value to be decremented.
 *   value - The 32-bit subtrahend
 *
 * Returned Value:
 *   The decremented value (volatile!)
 *
 ****************************************************************************/

up_fetchsub32:

	ldrex	r2, [r0]			/* Fetch the value to be decremented */
	sub		r2, r2, r1			/* Subtract the subtrahend */

	strex	r3, r2, [r0]		/* Attempt to save the result */
	teq		r3, #0				/* r3 will be 1 if strex failed */
	bne		up_fetchsub32		/* Failed to lock... try again */

	mov		r0, r2				/* Return the decremented value */
	bx		lr					/* Successful! */

/****************************************************************************
 * Name: up_fetchadd16
 *
 * Description:
 *   Perform an atomic fetch add operation on the provided 16-bit value.
 *
 *   This function must be provided via the architecture-specific logic.
 *
 * Input Parameters:
 *   addr  - The address of 16-bit value to be incremented.
 *   value - The 16-bit addend
 *
 * Returned Value:
 *   The incremented value (volatile!)
 *
 ****************************************************************************/

up_fetchadd16:

	ldrexh	r2, [r0]			/* Fetch the value to be incremented */
	add		r2, r2, r1			/* Add the addend */

	strexh	r3, r2, [r0]		/* Attempt to save the result */
	teq		r3, #0				/* r3 will be 1 if strexh failed */
	bne		up_fetchadd16		/* Failed to lock... try again */

	mov		r0, r2				/* Return the incremented value */
	bx		lr					/* Successful! */

/****************************************************************************
 * Name: up_fetchsub16
 *
 * Description:
 *   Perform an atomic fetch subtract operation on the provided 16-bit value.
 *
 *   This function must be provided via the architecture-specific logic.
 *
 * Input Parameters:
 *   addr  - The address of 16-bit value to be decremented.
 *   value - The 16-bit subtrahend
 *
 * Returned Value:
 *   The decremented value (volatile!)
 *
 ****************************************************************************/

up_fetchsub16:

	ldrexh	r2, [r0]			/* Fetch the value to be decremented */
	sub		r2, r2, r1			/* Subtract the subtrahend */

	/* Attempt to save the decremented value */

	strexh	r3, r2, [r0]		/* Attempt to save the result */
	teq		r3, #0				/* r3 will be 1 if strexh failed */
	bne		up_fetchsub16		/* Failed to lock... try again */

	mov		r0, r2				/* Return the decremented value */
	bx		lr					/* Successful! */

/****************************************************************************
 * Name: up_fetchadd8
 *
 * Description:
 *   Perform an atomic fetch add operation on the provided 8-bit value.
 *
 *   This function must be provided via the architecture-specific logic.
 *
 * Input Parameters:
 *   addr  - The address of 8-bit value to be incremented.
 *   value - The 8-bit addend
 *
 * Returned Value:
 *   The incremented value (volatile!)
 *
 ****************************************************************************/

up_fetchadd8:

	ldrexb	r2, [r0]			/* Fetch the value to be incremented */
	add		r2, r2, r1			/* Add the addend */

	strexb	r3, r2, [r0]		/* Attempt to save the result */
	teq		r3, #0				/* r3 will be 1 if strexb failed */
	bne		up_fetchadd8		/* Failed to lock... try again */

	mov		r0, r2				/* Return the incremented value */
	bx		lr					/* Successful! */

/****************************************************************************
 * Name: up_fetchsub8
 *
 * Description:
 *   Perform an atomic fetch subtract operation on the provided 8-bit value.
 *
 *   This function must be provided via the architecture-specific logic.
 *
 * Input Parameters:
 *   addr  - The address of 8-bit value to be decremented.
 *   value - The 8-bit subtrahend
 *
 * Returned Value:
 *   The decremented value (volatile!)
 *
 ****************************************************************************/

up_fetchsub8:

	ldrexb	r2, [r0]			/* Fetch the value to be decremented */
	sub		r2, r2, r1			/* Subtract the subtrahend */

	strexb	r3, r2, [r0]		/* Attempt to save the result */
	teq		r3, #0				/* r3 will be 1 if strexb failed */
	bne		up_fetchsub8		/* Failed to lock... try again */

	mov		r0, r2				/* Return the decremented value */
	bx		lr					/* Successful! */

	END

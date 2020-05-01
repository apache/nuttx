/****************************************************************************
 * arch/arm/src/armv7-r/arm_fetchadd.S
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

	.file	"arm_fetchadd.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

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

	.globl	up_fetchadd32
	.type	up_fetchadd32, %function

up_fetchadd32:

1:
	ldrex	r2, [r0]			/* Fetch the value to be incremented */
	add		r2, r2, r1			/* Add the addend */

	strex	r3, r2, [r0]		/* Attempt to save the result */
	teq		r3, #0				/* r2 will be 1 is strex failed */
	bne		1b					/* Failed to lock... try again */

	mov		r0, r2				/* Return the incremented value */
	bx		lr					/* Successful! */
	.size	up_fetchadd32, . - up_fetchadd32

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

	.globl	up_fetchsub32
	.type	up_fetchsub32, %function

up_fetchsub32:

1:
	ldrex	r2, [r0]			/* Fetch the value to be decremented */
	sub		r2, r2, r1			/* Subtract the subtrahend */

	strex	r3, r2, [r0]		/* Attempt to save the result */
	teq		r3, #0				/* r2 will be 1 is strex failed */
	bne		1b					/* Failed to lock... try again */

	mov		r0, r2				/* Return the decremented value */
	bx		lr					/* Successful! */
	.size	up_fetchsub32, . - up_fetchsub32

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

	.globl	up_fetchadd16
	.type	up_fetchadd16, %function

up_fetchadd16:

1:
	ldrexh	r2, [r0]			/* Fetch the value to be incremented */
	add		r2, r2, r1			/* Add the addend */

	strexh	r3, r2, [r0]		/* Attempt to save the result */
	teq		r3, #0				/* r2 will be 1 is strexh failed */
	bne		1b					/* Failed to lock... try again */

	mov		r0, r2				/* Return the incremented value */
	bx		lr					/* Successful! */
	.size	up_fetchadd16, . - up_fetchadd16

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

	.globl	up_fetchsub16
	.type	up_fetchsub16, %function

up_fetchsub16:

1:
	ldrexh	r2, [r0]			/* Fetch the value to be decremented */
	sub		r2, r2, r1			/* Subtract the subtrahend */

	/* Attempt to save the decremented value */

	strexh	r3, r2, [r0]		/* Attempt to save the result */
	teq		r3, #0				/* r2 will be 1 is strexh failed */
	bne		1b					/* Failed to lock... try again */

	mov		r0, r2				/* Return the decremented value */
	bx		lr					/* Successful! */
	.size	up_fetchsub16, . - up_fetchsub16

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

	.globl	up_fetchadd8
	.type	up_fetchadd8, %function

up_fetchadd8:

1:
	ldrexb	r2, [r0]			/* Fetch the value to be incremented */
	add		r2, r2, r1			/* Add the addend */

	strexb	r3, r2, [r0]		/* Attempt to save the result */
	teq		r3, #0				/* r2 will be 1 is strexb failed */
	bne		1b					/* Failed to lock... try again */

	mov		r0, r2				/* Return the incremented value */
	bx		lr					/* Successful! */
	.size	up_fetchadd8, . - up_fetchadd8

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

	.globl	up_fetchsub8
	.type	up_fetchsub8, %function

up_fetchsub8:

1:
	ldrexb	r2, [r0]			/* Fetch the value to be decremented */
	sub		r2, r2, r1			/* Subtract the subtrahend */

	strexb	r3, r2, [r0]		/* Attempt to save the result */
	teq		r3, #0				/* r2 will be 1 is strexb failed */
	bne		1b					/* Failed to lock... try again */

	mov		r0, r2				/* Return the decremented value */
	bx		lr					/* Successful! */
	.size	up_fetchsub8, . - up_fetchsub8
	.end

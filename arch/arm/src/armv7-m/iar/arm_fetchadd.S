/****************************************************************************
 * arch/arm/src/armv7-m/iar/arm_fetchadd.S
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

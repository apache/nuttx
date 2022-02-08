/****************************************************************************
 * arch/arm/src/armv7-a/cp15_cache_size.S
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
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "cp15.h"

	.file	"cp15_cache_size.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	cp15_cache_size

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: cp15_cache_size
 *
 * Description:
 *   Get cp15 cache size in byte
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Cache size in byte
 *
 ****************************************************************************/

	.globl	cp15_cache_size
	.type	cp15_cache_size, function

cp15_cache_size:

	mrc		CP15_CCSIDR(r0)			/* Read the Cache Size Identification Register */

	ldr		r3, =0x7fff			/* Isolate the NumSets field (bits 13-27) */
	and		r2, r3, r0, lsr #13		/* r2=NumSets (number of sets - 1) */
	add		r2, #1

	ldr		r3, =0x3ff			/* Isolate the way field (bits 3-12) */
	and		r1, r3, r0, lsr #3		/* r1=(number of ways - 1) */
	add		r1, #1

	ldr		r3,=0x7				/* Isolate the LineSize field (bits 0-2) */
	and		r0, r3				/* r0=(Log2LineSize - 2) in word */
	add		r0, #4				/* r0=Log2lineSize in byte */

	mul		r2, r1, r2			/* r2=Sets*Ways */
	lsl		r0, r2, r0			/* r0=Sets*Ways*LineSize */

	bx		lr

	.size cp15_cache_size, . - cp15_cache_size
	.end

/****************************************************************************
 * arch/arm/src/armv7-a/cp15_flush_dcache_all.S
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Portions of this file derive from Atmel sample code for the SAMA5D3
 * Cortex-A5 which also has a modified BSD-style license:
 *
 *   Copyright (c) 2012, Atmel Corporation
 *   All rights reserved.
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
 * 3. Neither the name NuttX nor Atmel nor the names of the contributors may
 *    be used to endorse or promote products derived from this software
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

/* References:
 *
 *  "Cortex-A5 MPCore, Technical Reference Manual", Revision: r0p1,
 *   Copyright (c) 2010  ARM. All rights reserved. ARM DDI 0434B (ID101810)
 *  "ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition",
 *   Copyright (c) 1996-1998, 2000, 2004-2012 ARM. All rights reserved. ARM
 *   DDI 0406C.b (ID072512)
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "cp15.h"

	.file	"cp15_flush_dcache_all.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	cp15_flush_dcache_all

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: cp15_flush_dcache_all
 *
 * Description:
 *   Flush the entire contents of D cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.globl	cp15_flush_dcache_all
	.type	cp15_flush_dcache_all, function

cp15_flush_dcache_all:

	push		{r4, r5}

	mrc		CP15_CCSIDR(r1)			/* Read the Cache Size Identification Register */

	ldr		r3, =0x7fff			/* Isolate the NumSets field (bits 13-27) */
	and		r0, r3, r1, lsr #13		/* r0=NumSets (number of sets - 1) */

	ldr		r3,=0x7				/* Isolate the LineSize field (bits 0-2) */
	and		r5, r3, r1			/* r4=(Log2LineSize - 2) in word */
	add		r5, #4				/* r4=Set/way operation line shfit */

	ldr		r3, =0x3ff			/* Isolate the way field (bits 3-12) */
	and		r1, r3, r1, lsr #3		/* r1=(number of ways - 1) */

	add		r4, r1, #1
	clz		r4, r4
	add		r4, #1				/* r4=Set/way operation Way shift */

way_loop:
	mov		r3, r0				/* Init Sets */
set_loop:
	mov		r2, r1, lsl r4			/* r2 = way loop counter << way shift */
	orr		r2, r3, lsl r5			/* r2 = set/way cache operation format */
	mcr		CP15_DCCISW(r2)			/* Data Cache Clean and Invalidate by Set/Way */
	subs		r3, r3, #1			/* Subtraction set counter */
	bcs		set_loop			/* Keep looping if not */

	subs		r1, r1, #1			/* Subtraction the way counter */
	bcs		way_loop			/* Keep looping if not */

	dsb
	pop		{r4, r5}
	bx		lr
	.size cp15_flush_dcache_all, . - cp15_flush_dcache_all
	.end

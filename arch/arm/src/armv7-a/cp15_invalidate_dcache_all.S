/****************************************************************************
 * arch/arm/src/armv7-a/cp15_invalidate_dcache_all.S
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *  "Cortex-A5™ MPCore, Technical Reference Manual", Revision: r0p1,
 *   Copyright © 2010  ARM. All rights reserved. ARM DDI 0434B (ID101810)
 *  "ARM® Architecture Reference Manual, ARMv7-A and ARMv7-R edition",
 *   Copyright © 1996-1998, 2000, 2004-2012 ARM. All rights reserved. ARM
 *   DDI 0406C.b (ID072512)
 *
 * Portions of this file derive from Atmel sample code for the SAMA5D3 Cortex-A5
 * which also has a modified BSD-style license:
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "cp15.h"

	.file	"cp15_invalidate_dcache_all.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	cp15_invalidate_dcache_all

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: cp15_invalidate_dcache_all
 *
 * Description:
 *   Invalidate the entire contents of D cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.globl	cp15_invalidate_dcache_all
	.type	cp15_invalidate_dcache_all, function

cp15_invalidate_dcache_all:

	mrc		CP15_CCSIDR(r0)			/* Read the Cache Size Identification Register */
	ldr		r3, =0xffff				/* Isolate the NumSets field (bits 13-27) */
	and		r0, r3, r0, lsr #13		/* r0=NumSets (number of sets - 1) */

	mov		r1, #0					/* r1 = way loop counter */
way_loop:

	mov		r3, #0					/* r3 = set loop counter */
set_loop:
	mov		r2, r1, lsl #30			/* r2 = way loop counter << 30 */
	orr		r2, r3, lsl #5			/* r2 = set/way cache operation format */
	mcr		CP15_DCISW(r2)			/* Data Cache Invalidate by Set/Way */
	add		r3, r3, #1				/* Increment set counter */
	cmp		r0, r3					/* Last set? */
	bne		set_loop				/* Keep looping if not */

	add		r1, r1, #1				/* Increment the way counter */
	cmp		r1, #4					/* Last way? (four ways assumed) */
	bne		way_loop				/* Keep looping if not */

	dsb
	bx		lr
	.size cp15_invalidate_dcache_all, . - cp15_invalidate_dcache_all
	.end

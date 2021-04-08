/****************************************************************************
 * arch/arm/src/armv7-r/cp15_clean_dcache.S
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

	.file	"cp15_clean_dcache.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	cp15_clean_dcache

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: cp15_clean_dcache
 *
 * Description:
 *   Clean the data cache within the specified region by flushing the
 *   contents of the data cache to memory.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.globl	cp15_clean_dcache
	.type	cp15_clean_dcache, function

cp15_clean_dcache:

	mrc		CP15_CTR(r3)			/* Read the Cache Type Register */
	lsr		r3, r3, #16			/* Isolate the DMinLine field */
	and		r3, r3, #0xf
	mov		r2, #4
	mov		r2, r2, lsl r3			/* Get the cache line size in bytes */

	sub		r3, r2, #1			/* R3=Cache line size mask */
	bic		r0, r0, r3			/* R0=aligned start address */

	/* Loop, cleaning each cache line by writing its contents to memory */

1:
	mcr		CP15_DCCMVAC(r0)		/* Clean data cache line to PoC by VA */
	add		r0, r0, r2			/* R12=Next cache line */
	cmp		r0, r1				/* Loop until all cache lines have been cleaned */
	blo		1b

	dsb
	bx		lr
	.size cp15_clean_dcache, . - cp15_clean_dcache
	.end

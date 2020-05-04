/****************************************************************************
 * arch/arm/src/arm/arm_cache.S
 *
 *   Copyright (C) 2007, 2009, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Several of these cache operations come from Atmel sample code with
 * modifications for better integration with NuttX.  The Atmel sample code
 * has a BSD compatible license that requires this copyright notice:
 *
 *   Copyright (c) 2008, Atmel Corporation
 *
 * [Actually, I think that all of the Atmel functions are commented out now]
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
 * 3. Neither the names NuttX nor Atmel nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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

	.file	"arm_cache.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CACHE_DLINESIZE    32

/****************************************************************************
 * Cache Operations
 ****************************************************************************/

	.text

/* Control functions caches and the write buffer c7
 * Register c7 controls the caches and the write buffer. The function of each cache
 * operation is selected by the Opcode_2 and CRm fields in the MCR instruction used to
 * write to CP15 c7. Writing other Opcode_2 or CRm values is Unpredictable.
 * Reading from CP15 c7 is Unpredictable, with the exception of the two test and clean
 * operations (see Table 2-18 on page 2-21 and Test and clean operations on page 2-23).
 * You can use the following instruction to write to c7:
 * MCR p15, <Opcode_1>, <Rd>, <CRn>, <CRm>, <Opcode_2>
 *
 * Invalidate Icache and Dcache                        MCR p15, 0, <Rd>, c7, c7, 0
 * Invalidate Icache                                   MCR p15, 0, <Rd>, c7, c5, 0
 * Invalidate Icache single entry (MVA) MVA            MCR p15, 0, <Rd>, c7, c5, 1
 * Invalidate Icache single entry (Set/Way) Set/Way    MCR p15, 0, <Rd>, c7, c5, 2
 * Prefetch Icache line (MVA) MVA                      MCR p15, 0, <Rd>, c7, c13, 1
 * Invalidate Dcache                                   MCR p15, 0, <Rd>, c7, c6, 0
 * Invalidate Dcache single entry (MVA) MVA            MCR p15, 0, <Rd>, c7, c6, 1
 * Invalidate Dcache single entry (Set/Way) Set/Way    MCR p15, 0, <Rd>, c7, c6, 2
 * Clean Dcache single entry (MVA) MVA                 MCR p15, 0, <Rd>, c7, c10, 1
 * Clean Dcache single entry (Set/Way) Set/Way         MCR p15, 0, <Rd>, c7, c10, 2
 * Test and clean Dcache -                             MRC p15, 0, <Rd>, c7, c10, 3
 * Clean and invalidate Dcache entry (MVA)  MVA        MCR p15, 0, <Rd>, c7, c14, 1
 * Clean and invalidate Dcache entry (Set/Way) Set/Way MCR p15, 0, <Rd>, c7, c14, 2
 * Test, clean, and invalidate Dcache -                MRC p15, 0, <Rd>, c7, c14, 3
 * Drain write buffer SBZ                              MCR p15, 0, <Rd>, c7, c10, 4
 * Wait for interrupt SBZ                              MCR p15, 0, <Rd>, c7, c0, 4
 */

/* Esure coherency between the Icache and the Dcache in the region described
 * by r0=start and r1=end.  Cleans the corresponding D-cache lines and invalidates
 * the corresponding I-Cache lines.
 */

	.globl	up_coherent_dcache
	.type	up_coherent_dcache, function

up_coherent_dcache:
	bic		r0, r0, #CACHE_DLINESIZE - 1
1:	mcr		p15, 0, r0, c7, c10, 1		/* Clean D entry */
	mcr		p15, 0, r0, c7, c5, 1		/* Invalidate I entry */
	add		r0, r0, #CACHE_DLINESIZE
	cmp		r0, r1
	blo		1b
	mcr		p15, 0, r0, c7, c10, 4		/* Drain WB */
	mov		pc, lr
	.size	up_coherent_dcache, .-up_coherent_dcache

/* Invalidate ICache in the region described by r0=start and r1=end. */

	.globl	up_invalidate_icache
	.type	up_invalidate_icache, function

up_invalidate_icache:
	bic		r0, r0, #CACHE_DLINESIZE - 1
1:	mcr		p15, 0, r0, c7, c5, 1		/* Invalidate I entry */
	add		r0, r0, #CACHE_DLINESIZE
	cmp		r0, r1
	blo		1b
	mov		pc, lr
	.size	up_invalidate_icache, .-up_invalidate_icache

/* Invalidate all of Icache */

	.globl	up_invalidate_icache_all
	.type	up_invalidate_icache_all, function

up_invalidate_icache_all:
	mov		r0, #0
	mcr		p15, 0, r0, c7, c5, 0
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	bx		lr
	.size	up_invalidate_icache_all, . - up_invalidate_icache_all

/* Invalidate D-Cache in the region described by r0=start and r1=end. */

	.globl	up_invalidate_dcache
	.type	up_invalidate_dcache, function

up_invalidate_dcache:
	bic		r0, r0, #CACHE_DLINESIZE - 1
1:	mcr		p15, 0, r0, c7, c6, 1		/* Invalidate D entry */
	add		r0, r0, #CACHE_DLINESIZE
	cmp		r0, r1
	blo		1b
	mov		pc, lr
	.size	up_invalidate_dcache, .-up_invalidate_dcache

/* Invalidate Dcache */

	.globl	up_invalidate_dcache_all
	.type	up_invalidate_dcache_all, function

up_invalidate_dcache_all:
	mov		r0, #0
	mcr		p15, 0, r0, c7, c6, 0
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	bx		lr
	.size	up_invalidate_dcache_all, . - up_invalidate_dcache_all

/* Clean D-Cache in the region described by r0=start and r1=end. */

	.globl	up_clean_dcache
	.type	up_clean_dcache, function

up_clean_dcache:
	bic		r0, r0, #CACHE_DLINESIZE - 1
1:	mcr		p15, 0, r0, c7, c10, 1		/* Clean D entry */
	add		r0, r0, #CACHE_DLINESIZE
	cmp		r0, r1
	blo		1b
	mov		pc, lr
	.size	up_clean_dcache, .-up_clean_dcache

/* Clean D-cache */

	.globl	up_clean_dcache_all
	.type	up_clean_dcache_all, function

up_clean_dcache_all:
	mrc		p15, 0, r0, c7, c10, 3
	bne		up_clean_dcache_all
	bx		lr
	.size	up_clean_dcache_all, . - up_clean_dcache_all

/* Clean & invalidate D-Cache in the region described by r0=start and r1=end. */

	.globl	up_flush_dcache
	.type	up_flush_dcache, function

up_flush_dcache:
	bic		r0, r0, #CACHE_DLINESIZE - 1
1:	mcr		p15, 0, r0, c7, c14, 1		/* Clean & invalidate D entry */
	add		r0, r0, #CACHE_DLINESIZE
	cmp		r0, r1
	blo		1b
	mov		pc, lr
	.size	up_flush_dcache, .-up_flush_dcache

/* CP15 Test, clean, and invalidate Dcache c7
 * As for test and clean, except that when the entire cache has
 * been tested and cleaned, it is invalidated.
 */

	.globl	up_flush_dcache_all
	.type	up_flush_dcache_all, function

up_flush_dcache_all:
	mrc		p15, 0, r0, c7, c14, 3
	bne		up_flush_dcache_all
	bx		lr
	.size	up_flush_dcache_all, . - up_flush_dcache_all

	.end

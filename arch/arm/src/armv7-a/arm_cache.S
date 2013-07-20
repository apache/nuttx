/****************************************************************************
 * arch/arm/src/armv7-a/arm_cache.S
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

/* cp15_cache Cache Operations
 *
 * Usage
 *
 * They are performed as MCR instructions and only operate on a level 1 cache
 * associated with  ARM v7 processor.
 *
 * The supported operations are:
 *
 *   1. Any of these operations can be applied to any data cache or any
 *      unified cache.
 *   2. Invalidate by MVA. Performs an invalidate of a data or unified cache
 *      line
 *      based on the address it contains.
 *   3. Invalidate by set/way. Performs an invalidate of a data or unified
 *      cache line based on its location in the cache hierarchy.
 *   4. Clean by MVA.  Performs a clean of a data or unified cache line based
 *      on the address it contains.
 *   5. Clean by set/way. Performs a clean of a data or unified cache line
 *      based on its location in the cache hierarchy.
 *   6. Clean and Invalidate by MVA. Performs a clean and invalidate of a
 *      data or unified cache line based on the address it contains.
 *   7. Clean and Invalidate by set/way. Performs a clean and invalidate of
 *      a data or unified cache line based on its location in the cache
 *      hierarchy.
 *
 * NOTE: Many of these operations are implemented as assembly language
 * macros or as C inline functions in the file cache.h.  The larger functions
 * are implemented here as C-callable functions.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

	.file	"arm_cache.S"

/****************************************************************************
 * Preprocessor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	cp15_coherent_dcache_for_dma
	.globl	cp15_invalidate_dcache_for_dma
	.globl	cp15_clean_dcache_for_dma
	.globl	cp15_flush_dcache_for_dma
	.globl	cp15_flush_kern_dcache_for_dma

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: cp15_coherent_dcache_for_dma
 *
 * Description:
 *   Ensure that the I and D caches are coherent within specified region.
 *   This is typically used when code has been written to a memory region,
 *   and will be executed.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.globl	cp15_coherent_dcache_for_dma
	.type	cp15_coherent_dcache_for_dma, function

cp15_coherent_dcache_for_dma:

	mrc		p15, 0, r3, c0, c0, 1
	lsr		r3, r3, #16
	and		r3, r3, #0xf
	mov		r2, #4
	mov		r2, r2, lsl r3

	sub		r3, r2, #1
	bic		r12, r0, r3
1:
	mcr		p15, 0, r12, c7, c11, 1
	add		r12, r12, r2
	cmp		r12, r1
	blo		1b
	dsb

	mrc		p15, 0, r3, c0, c0, 1
	and		r3, r3, #0xf
	mov		r2, #4
	mov		r2, r2, lsl r3

	sub		r3, r2, #1
	bic		r12, r0, r3
2:
	mcr		p15, 0, r12, c7, c5, 1
	add		r12, r12, r2
	cmp		r12, r1
	blo		2b
	mov		r0, #0
	mcr		p15, 0, r0, c7, c1, 6
	mcr		 p15, 0, r0, c7, c5, 6
	dsb
	isb
	bx		lr
	.size cp15_coherent_dcache_for_dma, . - cp15_coherent_dcache_for_dma

/****************************************************************************
 * Name: cp15_invalidate_dcache_for_dma
 *
 * Description:
 *   Invalidate the data cache within the specified region; we will be
 *   performing a DMA operation in this region and we want to purge old data
 *   in the cache.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.globl	cp15_invalidate_dcache_for_dma
	.type	cp15_invalidate_dcache_for_dma, function

cp15_invalidate_dcache_for_dma:

	mrc		p15, 0, r3, c0, c0, 1
	lsr		r3, r3, #16
	and		r3, r3, #0xf
	mov		r2, #4
	mov		r2, r2, lsl r3

	sub		r3, r2, #1
	tst		r0, r3
	bic		r0, r0, r3

	mcrne	p15, 0, r0, c7, c14, 1

	tst		r1, r3
	bic		r1, r1, r3
	mcrne	p15, 0, r1, c7, c14, 1
3:
	mcr		p15, 0, r0, c7, c6, 1
	add		r0, r0, r2
	cmp		r0, r1
	blo		3b
	dsb
	bx		lr
	.size cp15_coherent_dcache_for_dma, . - cp15_coherent_dcache_for_dma

/****************************************************************************
 * Name: cp15_clean_dcache_for_dma
 *
 * Description:
 *   Clean the data cache within the specified region
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.globl	cp15_clean_dcache_for_dma
	.type	cp15_clean_dcache_for_dma, function

cp15_clean_dcache_for_dma:
	mrc		p15, 0, r3, c0, c0, 1
	lsr		r3, r3, #16
	and		r3, r3, #0xf
	mov		r2, #4
	mov		r2, r2, lsl r3

	sub		r3, r2, #1
	bic		r0, r0, r3
4:
	mcr		p15, 0, r0, c7, c10, 1
	add		r0, r0, r2
	cmp		r0, r1
	blo		4b
	dsb
	bx		lr
	.size cp15_clean_dcache_for_dma, . - cp15_clean_dcache_for_dma

/****************************************************************************
 * Name: cp15_flush_dcache_for_dma
 *
 * Description:
 *   Flush the data cache within the specified region
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.globl	cp15_flush_dcache_for_dma
	.type	cp15_flush_dcache_for_dma, function

cp15_flush_dcache_for_dma:
	mrc		p15, 0, r3, c0, c0, 1
	lsr		r3, r3, #16
	and		r3, r3, #0xf
	mov		r2, #4
	mov		r2, r2, lsl r3
	sub		r3, r2, #1
	bic		r0, r0, r3
5:
	mcr		p15, 0, r0, c7, c14, 1
	add		r0, r0, r2
	cmp		r0, r1
	blo		5b
	dsb
	bx		lr
	.size cp15_flush_dcache_for_dma, . - cp15_flush_dcache_for_dma

/****************************************************************************
 * Name: cp15_flush_kern_dcache_for_dma
 *
 * Description:
 *   Ensure that the data held in the page kaddr is written back to the page
 *   in question.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

	.globl	cp15_flush_kern_dcache_for_dma
	.type	cp15_flush_kern_dcache_for_dma, function

cp15_flush_kern_dcache_for_dma:
	mrc		p15, 0, r3, c0, c0, 1
	lsr		r3, r3, #16
	and		r3, r3, #0xf
	mov		r2, #4
	mov		r2, r2, lsl r3

	add		r1, r0, r1
	sub		r3, r2, #1
	bic		r0, r0, r3

	mcr		p15, 0, r0, c7, c14, 1
	add		r0, r0, r2
	cmp		r0, r1
	blo		1b
	dsb
	bx		lr
	.size cp15_flush_kern_dcache_for_dma, . - cp15_flush_kern_dcache_for_dma
	.end

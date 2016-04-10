/****************************************************************************
 * arch/arm/src/arm/up_cache.S
 *
 *   Copyright (C) 2007, 2009, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Several of these cache operations come from Atmel sample code with
 * modifications for better integration with NuttX.  The Atmel sample code
 * has a BSD compatibile license that requires this copyright notice:
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

	.file	"up_cp15.S"

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

	.globl	cp15_flush_idcache
	.type	cp15_flush_idcache, function

cp15_flush_idcache:
	bic		r0, r0, #CACHE_DLINESIZE - 1
1:	mcr		p15, 0, r0, c7, c10, 1		/* Clean D entry */
	mcr		p15, 0, r0, c7, c5, 1		/* Invalidate I entry */
	add		r0, r0, #CACHE_DLINESIZE
	cmp		r0, r1
	blo		1b
	mcr		p15, 0, r0, c7, c10, 4		/* Drain WB */
	mov		pc, lr
	.size	cp15_flush_idcache, .-cp15_flush_idcache

#if 0 /* Not used */
/* Invalidate all of Icache and Dcache */

	.globl	cp15_invalidate_idcache
	.type	cp15_invalidate_idcache, function

cp15_invalidate_idcache:
	mov		r0, #0
	mcr		p15, 0, r0, c7, c7, 0
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	bx		lr
	.size	cp15_invalidate_idcache, . - cp15_invalidate_idcache

/* Invalidate all of Icache */

	.globl	cp15_invalidate_icache
	.type	cp15_invalidate_icache, function

cp15_invalidate_icache:
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
	.size	cp15_invalidate_icache, . - cp15_invalidate_icache
#endif /* Not used */

/* Invalidate D-Cache in the region described by r0=start and r1=end. */

	.globl	cp15_invalidate_dcache
	.type	cp15_invalidate_dcache, function

cp15_invalidate_dcache:
	bic		r0, r0, #CACHE_DLINESIZE - 1
	mcr		p15, 0, r0, c7, c6, 1		/* Invalidate D entry */
	add		r0, r0, #CACHE_DLINESIZE
	cmp		r0, r1
	blo		1b
	mov		pc, lr
	.size	cp15_flush_idcache, .-cp15_flush_idcache

#if 0 /* Not used */
/* Invalidate Dcache */

	.globl	cp15_invalidate_dcache_all
	.type	cp15_invalidate_dcache_all, function

cp15_invalidate_dcache_all:
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
	.size	cp15_invalidate_dcache_all, . - cp15_invalidate_dcache_all

/* CP15 Prefetch Icache line c7
 * Performs an Icache lookup of the specified modified virtual address.
 * If the cache misses, and the region is cacheable, a linefill is performed.
 * Prefetch Icache line (MVA): MCR p15, 0, <Rd>, c7, c13, 1
 */

	.globl	cp15_prefetch_icacheline
	.type	cp15_prefetch_icacheline, function

cp15_prefetch_icacheline:
	mcr		p15, 0, r0, c7, c13, 1
	bx		lr
	.size	cp15_prefetch_icacheline, . - cp15_prefetch_icacheline

/* CP15 Test, clean, and invalidate Dcache c7
 * As for test and clean, except that when the entire cache has
 * been tested and cleaned, it is invalidated.
 */

	.globl	cp15_testcleaninvalidate_dcache
	.type	cp15_testcleaninvalidate_dcache, function

cp15_testcleaninvalidate_dcache:
	mrc		p15, 0, r0, c7, c14, 3
	bne		cp15_testcleaninvalidate_dcache
	bx		lr
	.size	cp15_testcleaninvalidate_dcache, . - cp15_testcleaninvalidate_dcache

/* CP15 Drain write buffer c7
 * This instruction acts as an explicit memory barrier. It drains
 * the contents of the write buffers of all memory stores
 * occurring in program order before this instruction is
 * completed. No instructions occurring in program order
 * after this instruction are executed until it completes. This
 * can be used when timing of specific stores to the level two
 * memory system has to be controlled (for example, when a
 * store to an interrupt acknowledge location has to complete
 * before interrupts are enabled).
 */

	.globl	cp15_drain_writebuffer
	.type	cp15_drain_writebuffer, function

cp15_drain_writebuffer:
	mov		r0, #0
	mcr		p15, 0, r0, c7, c10, 4
	bx		lr
	.size	cp15_drain_writebuffer, . - cp15_drain_writebuffer

/****************************************************************************
 * Cache Lockdown
 ****************************************************************************/

/* Cache Lockdown Register c9
 * The Cache Lockdown Register uses a cache-way-based locking scheme (Format C) that
 * enables you to control each cache way independently.
 * These registers enable you to control which cache ways of the four-way cache are used
 * for the allocation on a linefill. When the registers are defined, subsequent linefills are
 * only placed in the specified target cache way. This gives you some control over the
 * cache pollution caused by particular applications, and provides a traditional lockdown
 * operation for locking critical code into the cache.
 *
 * Read Dcache Lockdown Register   MRC p15,0,<Rd>,c9,c0,0
 * Write Dcache Lockdown Register  MCR p15,0,<Rd>,c9,c0,0
 * Read Icache Lockdown Register   MRC p15,0,<Rd>,c9,c0,1
 * Write Icache Lockdown Register  MCR p15,0,<Rd>,c9,c0,1
 */

	.globl	cp15_read_dcachelockdown
	.type	cp15_read_dcachelockdown, function

cp15_read_dcachelockdown:
	mov		r0, #0
	mrc		p15, 0, r0, c9, c0, 0
	bx		lr
	.size	cp15_read_dcachelockdown, . - cp15_read_dcachelockdown

	.globl	cp15_write_dcachelockdown
	.type	cp15_write_dcachelockdown, function

cp15_write_dcachelockdown:
	mcr	p15, 0, r0, c9, c0, 0
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	bx	lr
	.size	cp15_write_dcachelockdown, . - cp15_write_dcachelockdown

	.globl	cp15_read_icachelockdown
	.type	cp15_read_icachelockdown, function

cp15_read_icachelockdown:
	mov		r0, #0
	mrc		p15, 0, r0, c9, c0, 1
	bx		lr
	.size	cp15_read_icachelockdown, . - cp15_read_icachelockdown

	.globl	cp15_write_icachelockdown
	.type	cp15_write_icachelockdown, function

cp15_write_icachelockdown:
	mcr		p15, 0, r0, c9, c0, 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	bx		lr
	.size	cp15_write_icachelockdown, . - cp15_write_icachelockdown
#endif /* Not used */
	.end

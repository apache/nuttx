/****************************************************************************
 * arch/arm/src/arm/pg_macros.S
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_ARM_PG_MACROS_H
#define __ARCH_ARM_SRC_ARM_PG_MACROS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/* Write one L2 entry for a coarse page table entry.
 *
 * Inputs (unmodified):
 *   ctab  - Register containing the address of the coarse page table
 *   paddr - Physical address of the page to be mapped
 *   vaddr - Virtual address of the page to be mapped
 *   mmuflags - the MMU flags to use in the mapping
 *
 * Scratch registers (modified): tmp1, tmp2
 */
 
	.macro	wrl2coarse, ctab, paddr, vaddr, mmuflags, tmp1, tmp2
	
	/* Get tmp1 = (paddr | mmuflags), the value to write into the table */

	orr	\tmp1, \mmuflags, \paddr

	/* index  = (vaddr & 0x000ff000) >> 12
	 * offset = (vaddr & 0x000ff000) >> 10
	 */

	and	\tmp2, \vaddr, #0x0000ff000

	/* Write value into table at ofset */

	str	\tmp1, [\ctab, \tmp2, lsr #10]
	.endm

/* Write one L1 entry for a coarse page table.
 *
 * Inputs (unmodified unless noted):
 *   paddr - Physical address of the section (modified)
 *   vaddr - Virtual address of the section
 *   mmuflags - MMU flags to use in the section mapping
 *
 * Scratch registers (modified): tmp1, tmp2, tmp3
 */

	.macro	wrl1coarse, paddr, vaddr, mmuflags, tmp1, tmp2
	/* tmp1 = the base of the L1 page table */

   	ldr	\tmp1, =PGTABLE_BASE_VADDR

	/* tmp2 = (paddr | mmuflags), the value to write into the page table */

	orr	\paddr, \paddr, \mmuflags

	/* Write the value into the table at the correc offset.
	 * table index = vaddr >> 20, offset = index << 2
	 */
	
	lsr	\tmp2, \vaddr, #20
	str	\paddr, [\tmp1, \tmp2, lsl #2]
	.endm

/* Write one coarse L1 entry and all assocated L2 entries for a
 * coarse page table.
 *
 * Inputs:
 *   offset - coarse page table offset (unmodified)
 *   paddr - Physical address of the section (modified)
 *   vaddr - Virtual address of the section (modified)
 *   npages - Number of pages to write in the section (modified)
 *
 * Scratch registers (modified): tmp1, tmp2, tmp3, tmp4, tmp5
 *
 * On return, paddr and vaddr refer to the beginning of the
 * next section.
 */
	.macro	wrcoarse, offset, paddr, vaddr, npages, tmp1, tmp2, tmp3, tmp4

	/* tmp1 = address of L2 table; tmp2 = MMU flags */

	ldr	\tmp1, =PGTABLE_COARSE_BASE_VADDR
	add	\tmp1, \offset, \paddr
	ldr	\tmp2, =MMU_L2_VECTORFLAGS
	b	2f
1:
	/* Write that L2 entry into the coarse page table */

	wrl2coarse \tmp1, \paddr, \vaddr, \tmp2, \tmp3, \tmp4

	/* Update the physical and virtual addresses that will
	 * correspond to the next table entry.
	 */

	add	\paddr, \paddr, #4096
	add	\vaddr, \vaddr, #4096
2:
	/* Check if all of the pages have been written.  If not, then
	 * loop and write the next entry.
	 */

	sub	\npages, \npages, #1
	cmn	\npages #1
	bne	1b

	/* Write the section entry that refers to this coarse page
	 * table.
	 */

	ldr	\tmp1, =PGTABLE_COARSE_BASE_PADDR
	ldr	\tmp2, =MMU_L1_VECTORFLAGS
	add	\tmp1, \offset, \tmp1
	wrl1coarse \tmp1, \vaddr, \tmp2, \tmp3, \tmp4
	.endm

/* Write several, contiguous coarse L1 page table entries (and all
 * associated L2 page table entries).  As many entries will be
 * written as many as needed to span npages.
 *
 * Inputs:
 *   offset - coarse page table offset (modified)
 *   paddr - Physical address of the section (modified)
 *   vaddr - Virtual address of the section
 *   npages - Number of pages to write in the section
 *
 * Scratch registers (modified): tmp1, tmp2, tmp3, tmp4, tmp5
 */

	.macro	wrsections, offset, paddr, vaddr, npages, tmp1, tmp2, tmp3, tmp4
	b	2f
1:
	/* Select the number of coarse, 4Kb pages to write in this section.
	 * This number will be 256 unless there are fewer than 256 pages
	 * remaining to be mapped.
	 */

	cmp	\npages, #255		/* Check if <= 255 */
	movls	\tmp1, \npages		/* YES.. tmp1 = npages */
	movls	\npages, #0		/*       npages = 0 */
	movhi	\tmp1, #256		/* NO..  tmp1 = 256 */
	subhi	\npages, \npages, #256	/*       npages -= 256 */

	/* Write the L2 entries for this section */

	wrcoarse \offset, \paddr, \vaddr, \tmp1, \tmp1, \tmp2, \tmp3, \tmp4
	add	\offset, \offset, #1024
2:
	cmp	\npages, #0
	bne	1b
	.endm
#endif /* __ARCH_ARM_SRC_ARM_PG_MACROS_H */

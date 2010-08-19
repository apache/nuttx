/****************************************************************************
 * arch/arm/src/arm/pg_macros.h
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
#include <nuttx/page.h>

#include "arm.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_PAGING
  /* Sanity check -- we cannot be using a ROM page table and supporting on-
   * demand paging.
   */
#  ifdef CONFIG_ARCH_ROMPGTABLE
#    error "Cannot support both CONFIG_PAGING and CONFIG_ARCH_ROMPGTABLE"
#  endif

  /* Create some friendly definitions to handle some differences between
   * small and tiny pages.
   */

#  if CONFIG_PAGING_PAGESIZE == 1024
#    define PTE_NPAGES PTE_TINY_NPAGES
#  elif CONFIG_PAGING_PAGESIZE == 4096
#    define PTE_NPAGES PTE_SMALL_NPAGES
#  else
#    error "Need extended definitions for CONFIG_PAGING_PAGESIZE"
#  endif

#define PT_SIZE (PTE_NPAGES * 4)
#endif /* CONFIG_PAGING */

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

#ifdef __ASSEMBLY

/****************************************************************************
 * Name: wrpte_coarse
 *
 * Description:
 *   Write one L2 entry for a coarse PTE.
 *
 * Inputs (unmodified):
 *   ctab  - Register containing the address of the coarse page table
 *   paddr - Physical address of the page to be mapped
 *   vaddr - Virtual address of the page to be mapped
 *   mmuflags - the MMU flags to use in the mapping
 *
 * Scratch registers (modified): tmp1, tmp2
 *
 ****************************************************************************/
 
#ifdef CONFIG_PAGING
	.macro	wrpte_coarse, ctab, paddr, vaddr, mmuflags, tmp1, tmp2
	
	/* Get tmp1 = (paddr | mmuflags), the value to write into the table */

	orr	\tmp1, \mmuflags, \paddr

	/* index  = (vaddr & 0x000ff000) >> 12
	 * offset = (vaddr & 0x000ff000) >> 10
	 */

	and	\tmp2, \vaddr, #0x0000ff000

	/* Write value into table at ofset */

	str	\tmp1, [\ctab, \tmp2, lsr #10]
	.endm
#endif /* CONFIG_PAGING */

/****************************************************************************
 * Name: wrpmd_coarse
 *
 * Description:
 *   Write one L1 entry for a coarse page table.
 *
 * Inputs (unmodified unless noted):
 *   paddr - Physical address of the section (modified)
 *   vaddr - Virtual address of the section
 *   mmuflags - MMU flags to use in the section mapping
 *
 * Scratch registers (modified): tmp1, tmp2, tmp3
 *
 ****************************************************************************/

#ifdef CONFIG_PAGING
	.macro	wrpmd_coarse, paddr, vaddr, mmuflags, tmp1, tmp2
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
#endif /* CONFIG_PAGING */

/****************************************************************************
 * Name: wr_coarse
 *
 * Description:
 *   Write one coarse L1 entry and all assocated L2 entries for a coarse
 *   page table.
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
 *
 ****************************************************************************/

#ifdef CONFIG_PAGING
	.macro	wr_coarse, offset, paddr, vaddr, npages, tmp1, tmp2, tmp3, tmp4

	/* tmp1 = address of L2 table; tmp2 = MMU flags */

	ldr	\tmp1, =PGTABLE_COARSE_BASE_VADDR
	add	\tmp1, \offset, \paddr
	ldr	\tmp2, =MMU_L2_VECTORFLAGS
	b	2f
1:
	/* Write that L2 entry into the coarse page table */

	wrpte_coarse \tmp1, \paddr, \vaddr, \tmp2, \tmp3, \tmp4

	/* Update the physical and virtual addresses that will
	 * correspond to the next table entry.
	 */

	add	\paddr, \paddr, #CONFIG_PAGING_PAGESIZE
	add	\vaddr, \vaddr, #CONFIG_PAGING_PAGESIZE
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
	wrpmd_coarse \tmp1, \vaddr, \tmp2, \tmp3, \tmp4
	.endm
#endif /* CONFIG_PAGING */

/****************************************************************************
 * Name: wr_sections
 *
 * Description:
 *   Write several, contiguous coarse L1 page table entries (and all
 *   associated L2 page table entries).  As many entries will be written as
 *   many as needed to span npages.
 *
 * Inputs:
 *   offset - coarse page table offset (modified)
 *   paddr - Physical address of the section (modified)
 *   vaddr - Virtual address of the section
 *   npages - Number of pages to write in the section
 *
 * Scratch registers (modified): tmp1, tmp2, tmp3, tmp4, tmp5
 *
 ****************************************************************************/

#ifdef CONFIG_PAGING
	.macro	wr_sections, offset, paddr, vaddr, npages, tmp1, tmp2, tmp3, tmp4
	b	2f
1:
	/* Select the number of pages to write in this section. This number
	 * will be 256 for coarse page tables or 1024 for fine/tiny page
	 * tables (unless the npages argument indicates that there are fewer
	 * than pages remaining to be mapped).
	 */

	cmp	\npages, #(PTE_NPAGES-1)	/* Check if npages < PTE_NPAGES */
	movls	\tmp1, \npages			/* YES.. tmp1 = npages */
	movls	\npages, #0			/*       npages = 0 */
	movhi	\tmp1, #PTE_NPAGES		/* NO..  tmp1 = PTE_NPAGES */
	subhi	\npages, \npages, #PTE_NPAGES	/*       npages -= PTE_NPAGES */

	/* Write the L2 entries for this section */

	wr_coarse \offset, \paddr, \vaddr, \tmp1, \tmp1, \tmp2, \tmp3, \tmp4
	add	\offset, \offset, #PT_SIZE
2:
	cmp	\npages, #0
	bne	1b
	.endm
#endif /* CONFIG_PAGING */

#endif /* __ASSEMBLY */
#endif /* __ARCH_ARM_SRC_ARM_PG_MACROS_H */

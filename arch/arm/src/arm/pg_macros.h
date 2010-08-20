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
#    define PTE_NPAGES         PTE_TINY_NPAGES
#    define PG_L2_BASE_PADDR   PGTABLE_FINE_BASE_PADDR
#    define PG_L2_BASE_VADDR   PGTABLE_FINE_BASE_VADDR
#    define MMU_L1_TEXTFLAGS   (PMD_TYPE_FINE|PMD_BIT4|PTE_CACHEABLE)
#    define MMU_L2_TEXTFLAGS   (PTE_TYPE_TINY|PTE_EXT_AP_UNO_SRO|PTE_CACHEABLE)
#    define MMU_L1_DATAFLAGS   (PMD_TYPE_FINE|PMD_BIT4|PTE_CACHEABLE)
#    define MMU_L2_DATAFLAGS   (PTE_TYPE_TINY|PTE_EXT_AP_UNO_SRW|PTE_CACHEABLE)
#  elif CONFIG_PAGING_PAGESIZE == 4096
#    define PTE_NPAGES PTE_SMALL_NPAGES
#    define PG_L2_BASE_PADDR   PGTABLE_COARSE_BASE_PADDR
#    define PG_L2_BASE_vADDR   PGTABLE_COARSE_BASE_VADDR
#    define MMU_L1_TEXTFLAGS   (PMD_TYPE_COARSE|PMD_BIT4|PTE_CACHEABLE)
#    define MMU_L2_TEXTFLAGS   (PTE_TYPE_SMALL|PTE_SMALL_AP_UNO_SRO|PTE_CACHEABLE)
#    define MMU_L1_DATAFLAGS   (PMD_TYPE_COARSE|PMD_BIT4|PTE_CACHEABLE|PTE_BUFFERABLE)
#    define MMU_L2_DATAFLAGS   (PTE_TYPE_SMALL|PTE_SMALL_AP_UNO_SRW|PTE_CACHEABLE|PTE_BUFFERABLE)
#  else
#    error "Need extended definitions for CONFIG_PAGING_PAGESIZE"
#  endif

#define PT_SIZE (PTE_NPAGES * 4)

/* We position the data section PTE's just after the text section PTE's */

#define PG_L2_DATA_PADDR       (PG_L2_BASE_PADDR + 4*PG_TEXT_NPAGES)
#endif /* CONFIG_PAGING */

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

#ifdef __ASSEMBLY

/****************************************************************************
 * Name: pg_map
 *
 * Description:
 *   Write several, contiguous L2 page table entries.  npages entries will be
 *   written. This macro is used when CONFIG_PAGING is enable.  This case,
 *   it is used asfollows:
 *
 *	ldr	r0, =PG_L2_BASE_PADDR
 *	ldr	r1, =PG_LOCKED_PBASE
 *	ldr	r2, =CONFIG_PAGING_NLOCKED
 *      ldr	r3, =MMUFLAGS
 *	pg_map r0, r1, r2, r3, r4
 *
 * Inputs:
 *   l2 - Physical start address in the L2 page table (modified)
 *   paddr - The physical address of the start of the region to span. Must
 *            be aligned to 1Mb section boundaries (modified)
 *   npages - Number of pages to write in the section (modified)
 *   mmuflags - L2 MMU FLAGS
 *
 * Scratch registers (modified): tmp
 *   l2  - Physical address in the L2 page table.
 *   tmp - scratch
 *
 * Assumptions:
 * - The MMU is not yet enabled
 * - The L2 page tables have been zeroed prior to calling this function
 * - pg_span has been called to initialize the L1 table.
 *
 ****************************************************************************/

#ifdef CONFIG_PAGING
	.macro	pg_map, paddr, npages, mmuflags, l2, tmp
	b	2f
1:
	/* Write the one L2 entries.  First,  get tmp = (paddr | mmuflags),
	 * the value to write into the L2 PTE
	 */

	orr	\tmp, \paddr, \mmuflags

	/* Write value into table at the current table address */

	str	\tmp, [\l2], #4

	/* Update the physical addresses that will correspond to the next
	 * table entry.
	 */

	add	\paddr, \paddr, #CONFIG_PAGING_PAGESIZE
	add	\l2, \l2, #4

	/* Decrement the number of pages written */

	sub	\npages, \npages, #1
2:
	/* Check if all of the pages have been written.  If not, then
	 * loop and write the next PTE.
	 */
	cmp	\npages, #0
	bgt	1b
	.endm

/****************************************************************************
 * Name: pg_span
 *
 * Description:
 *   Write several, contiguous unmapped coarse L1 page table entries.  As
 *   many entries will be written as  many as needed to span npages.  This
 *   macro is used when CONFIG_PAGING is enable.  This case, it is used as
 *   follows:
 *
 *	ldr	r0, =PG_L2_BASE_PADDR
 *	ldr	r1, =PG_LOCKED_PBASE
 *	ldr	r2, =(CONFIG_PAGING_NLOCKED+CONFIG_PAGING_NPAGES)
 *      ldr	r3, =MMU_FLAGS
 *	pg_span r0, r1, r2, r3, r4
 *
 * Inputs (unmodified unless noted):
 *   l2 - Physical start address in the L2 page table (modified)
 *   addr - The virtual address of the start of the region to span. Must
 *     be aligned to 1Mb section boundaries (modified)
 *   npages - Number of pages to required to span that memory region (modified)
 *   mmuflags - L1 MMU flags to use
 *
 * Scratch registers (modified):
 *   addr, npages, tmp
 *   l2  - L2 page table physical address
 *   addr - Physical address in the L1 page table.
 *   npages - The number of pages remaining to be accounted for
 *   tmp - scratch
 *
 * Return:
 *   Nothing of interest.
 *
 * Assumptions:
 * - The MMU is not yet enabled
 * - The L2 page tables have been zeroed prior to calling this function
 *
 ****************************************************************************/

#ifdef CONFIG_PAGING
	.macro	pg_span, l2, addr, npages, mmuflags, tmp

	/* Get addr = the L1 page table address coresponding to the virtual
	 * address of the start of memory region to be mapped.
	 */

   	ldr	\tmp, =PGTABLE_BASE_PADDR
	lsr	\addr, \addr, #20
	add	\addr, \tmp, \addr, lsl #2
	b	2f
1:
	/* Write the L1 table entry that refers to this (unmapped) coarse page
	 * table.
	 *
	 * tmp = (paddr | mmuflags), the value to write into the page table
	 */

	orr	\tmp, \l2, \mmuflags

	/* Write the value into the L1 table at the correct offset. */
	
	str	\tmp, [\addr], #4

	/* Update the L2 page table address for the next L1 table entry. */

	add	\l2, \l2, #PT_SIZE  /* Next L2 page table start paddr */

	/* Update the number of pages that we have account for (with
	 * non-mappings
	 */

	sub	\npages, \npages, PTE_NPAGES
2:
	/* Check if all of the pages have been written.  If not, then
	 * loop and write the next L1 entry.
	 */

	cmp	\npages, #0
	bgt	1b
	.endm

#endif /* CONFIG_PAGING */
#endif /* __ASSEMBLY */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY

#endif /* __ASSEMBLY */
#endif /* __ARCH_ARM_SRC_ARM_PG_MACROS_H */

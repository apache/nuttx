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

#ifdef CONFIG_ARCH_ROMPGTABLE
#  error "Cannot support both CONFIG_PAGING and CONFIG_ARCH_ROMPGTABLE"
#endif

/* Page Size Selections *****************************************************/

/* Create some friendly definitions to handle some differences between
 * small and tiny pages.
 */

#if CONFIG_PAGING_PAGESIZE == 1024

   /* Number of pages in an L2 table per L1 entry */

#  define PTE_NPAGES           PTE_TINY_NPAGES

   /* Mask to get the page table physical address from an L1 entry */

#  define PG_L1_PADDRMASK     PMD_FINE_TEX_MASK

   /* L2 Page table address */

#  define PG_L2_BASE_PADDR     PGTABLE_FINE_BASE_PADDR
#  define PG_L2_BASE_VADDR     PGTABLE_FINE_BASE_VADDR

   /* MMU Flags for each memory region */

#  define MMU_L1_TEXTFLAGS     (PMD_TYPE_FINE|PMD_BIT4)
#  define MMU_L2_TEXTFLAGS     (PTE_TYPE_TINY|PTE_EXT_AP_UNO_SRO|PTE_CACHEABLE)
#  define MMU_L1_DATAFLAGS     (PMD_TYPE_FINE|PMD_BIT4)
#  define MMU_L2_DATAFLAGS     (PTE_TYPE_TINY|PTE_EXT_AP_UNO_SRW|PTE_CACHEABLE|PTE_BUFFERABLE)
#  define MMU_L1_PGTABFLAGS    (PMD_TYPE_FINE|PMD_BIT4)
#  define MMU_L2_PGTABFLAGS    (PTE_TYPE_TINY|PTE_EXT_AP_UNO_SRW)

#  define MMU_L2_VECTRWFLAGS   (PTE_TYPE_TINY|PTE_EXT_AP_UNO_SRW)
#  define MMU_L2_VECTROFLAGS   (PTE_TYPE_TINY|PTE_EXT_AP_UNO_SRO|PTE_CACHEABLE)

#elif CONFIG_PAGING_PAGESIZE == 4096

   /* Number of pages in an L2 table per L1 entry */

#  define PTE_NPAGES           PTE_SMALL_NPAGES

   /* Mask to get the page table physical address from an L1 entry */

#  define PG_L1_PADDRMASK     PMD_COARSE_TEX_MASK

   /* L2 Page table address */

#  define PG_L2_BASE_PADDR     PGTABLE_COARSE_BASE_PADDR
#  define PG_L2_BASE_VADDR     PGTABLE_COARSE_BASE_VADDR

   /* MMU Flags for each memory region. */

#  define MMU_L1_TEXTFLAGS     (PMD_TYPE_COARSE|PMD_BIT4)
#  define MMU_L2_TEXTFLAGS     (PTE_TYPE_SMALL|PTE_SMALL_AP_UNO_SRO|PTE_CACHEABLE)
#  define MMU_L1_DATAFLAGS     (PMD_TYPE_COARSE|PMD_BIT4)
#  define MMU_L2_DATAFLAGS     (PTE_TYPE_SMALL|PTE_SMALL_AP_UNO_SRW|PTE_CACHEABLE|PTE_BUFFERABLE)
#  define MMU_L1_PGTABFLAGS    (PMD_TYPE_COARSE|PMD_BIT4)
#  define MMU_L2_PGTABFLAGS    (PTE_TYPE_SMALL|PTE_SMALL_AP_UNO_SRW)

#  define MMU_L2_VECTRWFLAGS   (PTE_TYPE_SMALL|PTE_SMALL_AP_UNO_SRW)
#  define MMU_L2_VECTROFLAGS   (PTE_TYPE_SMALL|PTE_SMALL_AP_UNO_SRO|PTE_CACHEABLE)

#else
#  error "Need extended definitions for CONFIG_PAGING_PAGESIZE"
#endif

#define PT_SIZE                (4*PTE_NPAGES)

/* We position the locked region PTEs at the beginning of L2 page
 * table.
 */

#define PG_L2_LOCKED_PADDR     PG_L2_BASE_PADDR
#define PG_L2_LOCKED_VADDR     PG_L2_BASE_VADDR
#define PG_L2_LOCKED_SIZE      (4*CONFIG_PAGING_NLOCKED)

/* We position the paged region PTEs immediately after the locked
 * region PTEs.
 */

#define PG_L2_PAGED_PADDR      (PG_L2_BASE_PADDR + PG_L2_LOCKED_SIZE)
#define PG_L2_PAGED_VADDR      (PG_L2_BASE_VADDR + PG_L2_LOCKED_SIZE)
#define PG_L2_PAGED_SIZE       (4*CONFIG_PAGING_NPPAGED)

/* This describes the overall text region */

#define PG_L2_TEXT_PADDR       PG_L2_LOCKED_PADDR
#define PG_L2_TEXT_VADDR       PG_L2_LOCKED_VADDR
#define PG_L2_TEXT_SIZE        (PG_L2_LOCKED_SIZE + PG_L2_PAGED_SIZE)

/* We position the data section PTEs just after the text region PTE's */

#define PG_L2_DATA_PADDR       (PG_L2_BASE_PADDR + PG_L2_TEXT_SIZE)
#define PG_L2_DATA_VADDR       (PG_L2_BASE_VADDR + PG_L2_TEXT_SIZE)
#define PG_L2_DATA_SIZE        (4*PG_DATA_NPAGES)

/* Page Table Info: The number of pages in the in the page table
 * (PG_PGTABLE_NPAGES).  We position the pagetable PTEs just after
 * the data section PTEs.
 */

#define PG_PGTABLE_NPAGES      (PGTABLE_SIZE >> PAGESHIFT)
#define PG_L2_PGTABLE_PADDR    (PG_L2_DATA_PADDR + PG_L2_DATA_SIZE)
#define PG_L2_PGTABLE_VADDR    (PG_L2_DATA_VADDR + PG_L2_DATA_SIZE)
#define PG_L2_PGTABLE_SIZE     (4*PG_DATA_NPAGES)

/* Vector mapping.  One page is required to map the vector table.  The
 * vector table could lie in at virtual address zero (or at the start
 * of RAM which is aliased to address zero on the ea3131) or at virtual
 * address 0xfff00000.  We only have logic here to support the former
 * case.
 *
 * NOTE:  If the vectors are at address zero, the page table will be
 * forced to the highest RAM addresses.  If the vectors are at 0xfff0000,
 * then the page table is forced to the beginning of RAM.
 *
 * When the vectors are at the beginning of RAM, they will probably overlap
 * the first page of the locked text region.  In any other case, the
 * configuration must set CONFIG_PAGING_VECPPAGE to provide the physical
 * address of the page to use for the vectors.
 *
 * When the vectors overlap the first page of the locked text region (the
 * only case in use so far), then the text page will be temporarily be made
 * writable in order to copy the vectors.
 *
 * PG_VECT_PBASE - This the physical address of the page in memory to be
 *   mapped to the vector address.
 * PG_L2_VECT_PADDR - This is the physical address of the L2 page table
 *   entry to use for the vector mapping.
 * PG_L2_VECT_VADDR - This is the virtual address of the L2 page table
 *   entry to use for the vector mapping.
 */

/* Case 1: The configuration tells us everything */

#if defined(CONFIG_PAGING_VECPPAGE)
#  define PG_VECT_PBASE          CONFIG_PAGING_VECPPAGE
#  define PG_L2_VECT_PADDR       CONFIG_PAGING_VECL2PADDR
#  define PG_L2_VECT_VADDR       CONFIG_PAGING_VECL2VADDR

/* Case 2: Vectors are in low memory and the locked text region starts at
 * the begin of SRAM (which will be aliased to address 0x00000000)
 */
 
#elif defined(CONFIG_ARCH_LOWVECTORS) && !defined(CONFIG_PAGING_LOCKED_PBASE)
#  define PG_VECT_PBASE          PG_LOCKED_PBASE
#  define PG_L2_VECT_PADDR       PG_L2_LOCKED_PADDR
#  define PG_L2_VECT_VADDR       PG_L2_LOCKED_VADDR

/* Case 3: High vectors or the locked region is not at the beginning or SRAM */

#else
#  error "Logic missing for high vectors in this case"
#endif

/* This is the total number of pages used in the text/data mapping: */

#define PG_TOTAL_NPPAGES         (PG_TEXT_NPPAGES + PG_DATA_PAGES + PG_PGTABLE_NPAGES)
#define PG_TOTAL_NVPAGES         (PG_TEXT_NVPAGES + PG_DATA_PAGES + PG_PGTABLE_NPAGES)
#if PG_TOTAL_NPPAGES >PG_RAM_PAGES
#  error "Total pages required exceeds RAM size"
#endif

/* For page managment purposes, the following summarize the "heap" of
 * free pages, operations on free pages and the L2 page table.
 *
 * PG_POOL_VA2L1OFFSET(va)  - Given a virtual address, return the L1 table
 *                            offset (in bytes).
 * PG_POOL_VA2L1VADDR(va)   - Given a virtual address, return the virtual
 *                            address of the L1 table entry
 * PG_POOL_L12PPTABLE(L1)   - Given the value of an L1 table entry return
 *                            the physical address of the start of the L2
 *                            page table
 * PG_POOL_L12PPTABLE(L1)   - Given the value of an L1 table entry return
 *                            the virtual address of the start of the L2
 *                            page table.
 *
 * PG_POOL_L1VBASE          - The virtual address of the start of the L1
 *                            page table range corresponding to the first
 *                            virtual address of the paged text region.
 * PG_POOL_L1VEND           - The virtual address of the end+1 of the L1
 *                            page table range corresponding to the last
 *                            virtual address+1 of the paged text region.
 *
 * PG_POOL_VA2L2NDX(va)     - Converts a virtual address within the paged
 *                            text region to the most compact possible
 *                            representation. Each PAGESIZE of address
 *                            corresponds to 1 index in the L2 page table;
 *                            Index 0 corresponds to the first L2 page table
 *                            entry for the first page in the virtual paged
 *                            text address space.
 * PG_POOL_NDX2VA(ndx)      - Performs the opposite conversion.. convests
 *                            an index into a virtual address in the paged
 *                            text region (the address at the beginning of
 *                            the page).
 * PG_POOL_MAXL2NDX         - This is the maximum value+1 of such an index.
 * 
 * PG_POOL_PGPADDR(ndx)     - Converts an page index into the corresponding
 *                            (physical) address of the backing page memory.
 * PG_POOL_PGVADDR(ndx)     - Converts an page index into the corresponding
 *                            (virtual)address of the backing page memory.
 *
 * These are used as follows:  If a miss occurs at some virtual address, va,
 * A new page index, ndx, is allocated.  PG_POOL_PGPADDR(i) converts the index
 * into the physical address of the page memory; PG_POOL_L2VADDR(va) converts
 * the virtual address in the L2 page table there the new mapping will be
 * written.
 */

#define PG_POOL_VA2L1OFFSET(va)  (((va) >> 20) << 2) 
#define PG_POOL_VA2L1VADDR(va)   (PGTABLE_BASE_VADDR + PG_POOL_VA2L1OFFSET(va))
#define PG_POOL_L12PPTABLE(L1)   ((L1) & PG_L1_PADDRMASK)
#define PG_POOL_L12VPTABLE(L1)   (PG_POOL_L12PPTABLE(L1) - PGTABLE_BASE_PADDR + PGTABLE_BASE_VADDR)

#define PG_POOL_L1VBASE          (PGTABLE_BASE_VADDR + ((PG_PAGED_VBASE >> 20) << 2))
#define PG_POOL_L1VEND           (PG_POOL_L1VBASE + (CONFIG_PAGING_NVPAGED << 2))

#define PG_POOL_VA2L2NDX(va)     (((va) -  PG_PAGED_VBASE) >> PAGESHIFT)
#define PG_POOL_NDX2VA(ndx)      (((ndx) << PAGESHIFT) + PG_PAGED_VBASE)
#define PG_POOL_MAXL2NDX         PG_POOL_VA2L2NDX(PG_PAGED_VEND)

#define PG_POOL_PGPADDR(ndx)     (PG_PAGED_PBASE + ((ndx) << PAGESHIFT))
#define PG_POOL_PGVADDR(ndx)     (PG_PAGED_VBASE + ((ndx) << PAGESHIFT))

#endif /* CONFIG_PAGING */

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__

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
#endif /* CONFIG_PAGING */

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
 *   l2 - L2 page table physical address
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
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_ARM_PG_MACROS_H */

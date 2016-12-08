/****************************************************************************
 * arch/arm/src/armv7-a/arm_pghead.S
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/page.h>

#include <arch/board/board.h>

#include "arm.h"
#include "cp15.h"
#include "sctlr.h"
#include "mmu.h"
#include "pg_macros.h"

#include "chip.h"
#include "up_internal.h"

	.file	"arm_pghead.S"

/**********************************************************************************
 * Configuration
 **********************************************************************************/
/* Assume these are not needed */

#undef ALIGNMENT_TRAP
#undef CPU_CACHE_ROUND_ROBIN
#undef CPU_DCACHE_DISABLE
#undef CPU_ICACHE_DISABLE

/* The page table cannot be in ROM if we are going to do pagin! */

#ifndef CONFIG_ARCH_ROMPGTABLE
#  error CONFIG_PAGING and CONFIG_ARCH_ROMPGTABLE are incompatible options
#endif

/* There are three operational memory configurations:
 *
 * 1. We execute in place in FLASH (CONFIG_BOOT_RUNFROMFLASH=y).  In this case
 *    the boot logic must:
 *
 *    - Configure SDRAM,
 *    - Initialize the .data section in RAM, and
 *    - Clear .bss section
 */

#ifdef CONFIG_BOOT_RUNFROMFLASH
  /* Check for the identity mapping:  For this configuration, this would be
   * the case where the virtual beginning of FLASH is the same as the physical
   * beginning of FLASH.
   */

#  if !defined(CONFIG_FLASH_START) || !defined(CONFIG_FLASH_VSTART)
#    error "CONFIG_FLASH_START or CONFIG_FLASH_VSTART is not defined"
#  endif

#  if CONFIG_FLASH_START == CONFIG_FLASH_VSTART
#    define CONFIG_IDENTITY_TEXTMAP 1
#  endif

/* 2. We boot in FLASH but copy ourselves to DRAM from better performance.
 *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=y).  In this case
 *    the boot logic must:
 *
 *    - Configure SDRAM,
 *    - Copy ourself to DRAM (after mapping it), and
 *    - Clear .bss section
 *
 *   In this case, we assume that the logic within this file executes from FLASH.
 */

#elif defined(CONFIG_BOOT_COPYTORAM)
#  error "configuration not implemented

  /* Check for the identity mapping:  For this configuration, this would be
   * the case where the virtual beginning of FLASH is the same as the physical
   * beginning of FLASH.
   */

#  if !defined(CONFIG_FLASH_START) || !defined(CONFIG_FLASH_VSTART)
#    error "CONFIG_FLASH_START or CONFIG_FLASH_VSTART is not defined"
#  endif

#  if CONFIG_FLASH_START == CONFIG_FLASH_VSTART
#    define CONFIG_IDENTITY_TEXTMAP 1
#  endif

/* 3. There is bootloader that copies us to DRAM (but probably not to the beginning)
 *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=n). In this case SDRAM
 *    was initialized by the boot loader, and this boot logic must:
 *
 *    - Clear .bss section
 */

#else

  /* Check for the identity mapping:  For this configuration, this would be
   * the case where the virtual beginning of RAM is the same as the physical
   * beginning of RAM.
   */

#  if !defined(CONFIG_RAM_START) || !defined(CONFIG_RAM_VSTART)
#    error "CONFIG_RAM_START or CONFIG_RAM_VSTART is not defined"
#  endif

#  if CONFIG_RAM_START == CONFIG_RAM_VSTART
#    define CONFIG_IDENTITY_TEXTMAP 1
#  endif

#endif

/* For each page table offset, the following provide (1) the physical address of
 * the start of the page table and (2) the number of page table entries in the
 * first page table.
 *
 * Coarse: PG_L1_PADDRMASK=0xfffffc00
 *         NPAGE1=(256 -((a) & 0x000003ff) >> 2) NPAGE1=1-256
 * Fine:   PG_L1_PADDRMASK=0xfffff000
 *         NPAGE1=(1024 -((a) & 0x00000fff) >> 2) NPAGE1=1-1024
 */

#define PG_L2_TEXT_PBASE     (PG_L2_TEXT_PADDR & PG_L1_PADDRMASK)
#define PG_L2_TEXT_NPAGE1    (PTE_NPAGES - ((PG_L2_TEXT_PADDR & ~PG_L1_PADDRMASK) >> 2))
#define PG_L2_PGTABLE_PBASE  (PG_L2_PGTABLE_PADDR & PG_L1_PADDRMASK)
#define PG_L2_PGTABLE_NPAGE1 (PTE_NPAGES - ((PG_L2_PGTABLE_PADDR & ~PG_L1_PADDRMASK) >> 2))
#define PG_L2_DATA_PBASE     (PG_L2_DATA_PADDR & PG_L1_PADDRMASK)
#define PG_L2_DATA_NPAGE1    (PTE_NPAGES - ((PG_L2_DATA_PADDR & ~PG_L1_PADDRMASK) >> 2))

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RX_NSECTIONS determines the number of 1Mb sections to map for the
 * Read/eXecute address region.  This is based on NUTTX_TEXT_SIZE.
 */

#define RX_NSECTIONS ((NUTTX_TEXT_SIZE+0x000fffff) >> 20)
#define WR_NSECTIONS ((NUTTX_RAM_SIZE+0x000fffff) >> 20)

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/* The ARMv7-A L1 page table can be placed at the beginning or at the end of
 * the RAM space.  This decision is based on the placement of the vector
 * area: If the vectors are place in low memory at address 0x0000 0000, then
 * the page table is placed in high memory; if the vectors are placed in
 * high memory at address 0xfff0 0000, then the page table is locating at
 * the beginning of RAM.
 *
 * For the special case where (1) the program executes out of RAM, and (2)
 * the page is located at the beginning of RAM (i.e., the high vector case),
 * then the following macro can easily find the physical address of the
 * section that includes the first part of the text region:  Since the page
 * table is closely related to the NuttX base address in this case, we can
 * convert the page table base address to the base address of the section
 * containing both.
 */

/* This macro will modify r0, r1, r2 and r14 */

#ifdef CONFIG_DEBUG_FEATURES
	.macro	showprogress, code
	mov		r0, #\code
	bl		up_lowputc
	.endm
#else
	.macro	showprogress, code
	.endm
#endif

/****************************************************************************
 * Name: __start
 ****************************************************************************/

	.text
	.global	__start
	.type	__start, #function

__start:
	/* Make sure that we are in SVC mode with IRQs and FIQs disabled */

	mov		r0, #(PSR_MODE_SVC | PSR_I_BIT | PSR_F_BIT)
	msr		cpsr_c, r0

	/* Clear the 16K level 1 page table */

	ldr		r4, .LCppgtable			/* r4=phys. page table */
	mov		r0, r4
	mov		r1, #0
	add		r2, r0, #PGTABLE_SIZE
.Lpgtableclear:
	str		r1, [r0], #4
	str		r1, [r0], #4
	str		r1, [r0], #4
	str		r1, [r0], #4
	teq		r0, r2
	bne		.Lpgtableclear

#ifdef ARMV7A_PGTABLE_MAPPING
	/* If the page table does not lie in the same address space as does the
	 * mapped RAM in either case.  So we will need to create a special
	 * mapping for the page table.
	 *
	 * Load information needed to map the page table.  After the ldmia, we
	 * will have
	 *
	 *   R1 = The aligned, physical base address of the page table
	 *   R2 = The aligned, virtual base address of the page table
	 *   R3 = The MMU flags to use with the .text space mapping
	 *   R5 = The physical address of the L1 page table (from above)
	 *
	 * The value in R1 could have been obtained by simply masking R5.
	 */

	adr		r0, .LCptinfo			/* Address of page table description */
	ldmia	r0, {r1, r2, r3}		/* Load the page table description */

	/* A single page is sufficient to map the page table */

	orr		r0, r1, r3				/* OR MMU flags into physical address */
	str		r0, [r5, r2, lsr #18]	/* Map using the virtual address as an index */
#endif

#ifndef CONFIG_IDENTITY_TEXTMAP
	/* Create identity mapping for first MB of the .text section to support
	 * this start-up logic executing out of the physical address space.  This
	 * identity mapping will be removed by .Lvstart (see below).  Of course,
	 * we would only do this if the physical-virtual mapping is not already
	 * the identity mapping.
	 */

	ldr		r0, .LCptextbase		/* r0=phys. base address of .text section */
	ldr		r1, .LCtextflags		/* R1=.text section MMU flags */
	orr		r3, r1, r0				/* r3=flags + base */
	str		r3, [r4, r0, lsr #18]	/* identity mapping */
#endif

	/* Map the read-only .text region in place.  This must be done
	 * before the MMU is enabled and the virtual addressing takes
	 * effect.  First populate the L1 table for the locked and paged
	 * text regions.
	 *
	 * We could probably make the pg_l1span and pg_l2map macros into
	 * call-able subroutines, but we would have to be carefully during
	 * this phase while we are operating in a physical address space.
	 *
	 * NOTE: That the value of r5 (L1 table base address) must be
	 * preserved through the following.
	 */

	adr		r0, .Ltxtspan
	ldmia	r0, {r0, r1, r2, r3, r5}
	pg_l1span r0, r1, r2, r3, r5, r6

	/* Then populate the L2 table for the locked text region only. */

	adr		r0, .Ltxtmap
	ldmia	r0, {r0, r1, r2, r3}
	pg_l2map r0, r1, r2, r3, r5

	/* Make sure that the page table is itself mapped and and read/write-able.
	 * First, populate the L1 table:
	 */

	adr		r0, .Lptabspan
	ldmia	r0, {r0, r1, r2, r3, r5}
	pg_l1span r0, r1, r2, r3, r5, r6

	/* Then populate the L2 table. */

	adr		r0, .Lptabmap
	ldmia	r0, {r0, r1, r2, r3}
	pg_l2map r0, r1, r2, r3, r5

	/* The following logic will set up the ARMv7-A for normal operation.
	 *
	 * Here we expect to have:
	 *   r4 = Address of the base of the L1 table
	 */

	/* Invalidate caches and TLBs.
	 *
	 *   NOTE: "The ARMv7 Virtual Memory System Architecture (VMSA) does not
	 *   support a CP15 operation to invalidate the entire data cache. ...
	 *   In normal usage the only time the entire data cache has to be
	 *   invalidated is on reset."
	 *
	 * The instruction cache is virtually indexed and physically tagged but
	 * the data cache is physically indexed and physically tagged.  So it
	 * should not be an issue if the system comes up with a dirty Dcache;
	 * the ICache, however, must be invalidated.
	 */

	mov		r0, #0
	mcr		CP15_TLBIALL(r0,c7)	/* Invalidate the entire unified TLB */
	mcr		CP15_TLBIALL(r0,c6)
	mcr		CP15_TLBIALL(r0,c5)
	mcr		CP15_BPIALL(r0)		/* Invalidate entire branch prediction array */
	mcr		CP15_ICIALLU(r0)	/* Invalidate I-cache */

	/* Load the page table address.
	 *
	 * NOTES:
	 * - Here we assume that the page table address is aligned to at least
	 *   least a 16KB boundary (bits 0-13 are zero).  No masking is provided
	 *   to protect against an unaligned page table address.
	 * - The Cortex-A5 has two page table address registers, TTBR0 and 1.
	 *   Only TTBR0 is used in this implementation but both are initialized.
	 *
	 * Here we expect to have:
	 *   r0 = Zero
	 *   r4 = Address of the base of the L1 table
	 */

	orr		r1, r4, #0x48
	mcr		CP15_TTBR0(r1)
	mcr		CP15_TTBR1(r1)

	/* Set the TTB control register (TTBCR) to indicate that we are using
	 * TTBR0.  r0 still holds the value of zero.
	 *
	 *   N   : 0=Selects TTBR0 and 16KB page table size indexed by VA[31:20]
	 *   PD0 : 0=Perform translation table walks using TTBR0
	 *   PD1 : 0=Perform translation table walks using TTBR1 (but it is disabled)
	 *   EAE : 0=Use 32-bit translation system
	 */

	mcr		CP15_TTBCR(r0)

	/* Enable the MMU and caches
	 * lr = Resume at .Lvstart with the MMU enabled
	 */

	ldr		lr, .LCvstart			/* Abs. virtual address */

	/* Configure the domain access register (see mmu.h).  Only domain 0 is
	 * supported and it uses the permissions in the TLB.
	 */

	mov		r0, #DACR_CLIENT(0)
	mcr		CP15_DACR(r0)			/* Set domain access register */

	/* Configure the system control register (see sctrl.h) */

	mrc		CP15_SCTLR(r0)			/* Get control register */

	/* Clear bits to reset values.  This is only necessary in situations like, for
	 * example, we get here via a bootloader and the control register is in some
	 * unknown state.
	 *
	 *   SCTLR_A    Bit 1:  Strict alignment disabled (reset value)
	 *   SCTLR_C    Bit 2:  DCache disabled (reset value)
	 *
	 *   SCTLR_SW   Bit 10: SWP/SWPB not enabled (reset value)
	 *   SCTLR_I    Bit 12: ICache disabled (reset value)
	 *   SCTLR_V    Bit 13: Assume low vectors (reset value)
	 *   SCTLR_RR   Bit 14: The Cortex-A5 processor only supports a fixed random
	 *                      replacement strategy.
	 *   SCTLR_HA   Bit 17: Not supported by A5
	 *
	 *   SCTLR_EE   Bit 25: Little endian (reset value).
	 *   SCTLR_TRE  Bit 28: No memory region remapping (reset value)
	 *   SCTLR_AFE  Bit 29: Full, legacy access permissions behavior (reset value).
	 *   SCTLR_TE   Bit 30: All exceptions handled in ARM state (reset value).
	 */

	bic		r0, r0, #(SCTLR_A  | SCTLR_C)
	bic		r0, r0, #(SCTLR_SW | SCTLR_I   | SCTLR_V   | SCTLR_RR | SCTLR_HA)
	bic		r0, r0, #(SCTLR_EE | SCTLR_TRE | SCTLR_AFE | SCTLR_TE)

	/* Set bits to enable the MMU
	 *
	 *   SCTLR_M     Bit 0:  Enable the MMU
	 *   SCTLR_Z     Bit 11: Program flow prediction control always enabled on A5
	 */

	orr		r0, r0, #(SCTLR_M)
#ifndef CONFIG_ARCH_CORTEXA5
	orr		r0, r0, #(SCTLR_Z)
#endif

#ifndef CONFIG_ARCH_LOWVECTORS
	/* Position vectors to 0xffff0000 if so configured.
	 *
	 *   SCTLR_V    Bit 13: High vectors
	 */

	orr		r0, r0, #(SCTLR_V)
#endif

#if defined(CPU_CACHE_ROUND_ROBIN) && !defined(CONFIG_ARCH_CORTEXA5)
	/* Round Robin cache replacement
	 *
	 *   SCTLR_RR   Bit 14: The Cortex-A5 processor only supports a fixed random
	 *                      replacement strategy.
	 */

	orr		r0, r0, #(SCTLR_RR)
#endif

	/* In SMP configurations, the data cache will not be enabled until later
	 * after SMP cache coherency has been setup.
	*/

#if !defined(CPU_DCACHE_DISABLE) && !defined(CONFIG_SMP)
	/* Dcache enable
	 *
	 *   SCTLR_C    Bit 2:  DCache enable
	 */

	orr		r0, r0, #(SCTLR_C)
#endif

#ifndef CPU_ICACHE_DISABLE
	/* Icache enable
	 *
	 *   SCTLR_I    Bit 12: ICache enable
	 */

	orr		r0, r0, #(SCTLR_I)
#endif

#ifdef ALIGNMENT_TRAP
	/* Alignment abort enable
	 *
	 *   SCTLR_A    Bit 1:  Strict alignment enabled
	 */

	orr		r0, r0, #(SCTLR_A)
#endif

#ifdef CPU_AFE_ENABLE
	/* AP[0:2] Permissions model
	 *
	 *   SCTLR_AFE  Bit 29: Full, legacy access permissions behavior (reset value).
	 *
	 * When AFE=1, the page table AP[0] is used as an access flag and AP[2:1]
	 * control.  When AFE=0, AP[2:0] control access permissions.
	 */

	orr		r0, r0, #(SCTLR_AFE)
#endif

	/* Then write the configured control register */

	mcr		CP15_SCTLR(r0)			/* Write control reg */
	.rept	12						/* Cortex A8 wants lots of NOPs here */
	nop
	.endr

	/* And "jump" to .Lvstart in the newly mapped virtual address space */

	mov		pc, lr

/****************************************************************************
 * PC_Relative Data
 ****************************************************************************/

	/* The virtual start address of the second phase boot logic */

	.type	.LCvstart, %object
.LCvstart:
	.long	.Lvstart
	.size	.LCvstart, . -.LCvstart

#ifdef ARMV7A_PGTABLE_MAPPING
	/* Page table region description.  The order of these fields must not
	 * change because the values are loaded using ldmia:
	 *
	 * 1) The aligned, physical base address of the page table
	 * 2) The aligned, virtual base address of the page table
	 * 3) The MMU flags to use with the .text space mapping
	 */

	.type	.LCptinfo, %object
.LCptinfo:
	.long	(PGTABLE_BASE_PADDR & 0xfff00000)	/* Physical base address */
	.long	(PGTABLE_BASE_VADDR & 0xfff00000)	/* Virtual base address */
	.long	MMU_MEMFLAGS						/* MMU flags for text section in RAM */
	.size	.LCptinfo, . -.LCptinfo
#endif

	/* The aligned, physical base address of the .text section */

	.type	.LCptextbase, %object
.LCptextbase:
	.long	NUTTX_TEXT_PADDR & 0xfff00000
	.size	.LCptextbase, . -.LCptextbase

	/* The aligned, virtual base address of the .text section */

	.type	.LCvtextbase, %object
.LCvtextbase:
	.long	NUTTX_TEXT_VADDR & 0xfff00000
	.size	.LCvtextbase, . -.LCvtextbase

	/* The MMU flags used with the .text mapping */

	.type	.LCtextflags, %object
.LCtextflags:
#ifdef CONFIG_BOOT_RUNFROMFLASH
	.long	MMU_ROMFLAGS			/* MMU flags text section in FLASH/ROM */
#else
	.long	MMU_MEMFLAGS			/* MMU flags for text section in RAM */
#endif
	.size	.LCtextflags, . -.LCtextflags

	/* The physical base address of the page table */

	.type	.LCppgtable, %object
.LCppgtable:
	.long	PGTABLE_BASE_PADDR		/* Physical start of page table */
	.size	.LCppgtable, . -.LCppgtable

	/* The virtual base address of the page table */

	.type	.LCvpgtable, %object
.LCvpgtable:
	.long	PGTABLE_BASE_VADDR		/* Virtual start of page table */
	.size	.LCvpgtable, . -.LCvpgtable

	.type	.Ltxtspan, %object
.Ltxtspan:
	.long	PG_L1_TEXT_PADDR		/* Physical address in the L1 table */
	.long	PG_L2_TEXT_PBASE		/* Physical address of the start of the L2 page table */
	.long	PG_TEXT_NVPAGES			/* Total (virtual) text pages to be mapped */
	.long	PG_L2_TEXT_NPAGE1		/* The number of text pages in the first page table */
	.long	MMU_L1_TEXTFLAGS		/* L1 MMU flags to use */
	.size	.Ltxtspan, . -.Ltxtspan

	.type	.Ltxtmap, %object
.Ltxtmap:
	.long	PG_L2_LOCKED_PADDR		/* Physical address in the L2 table */
	.long	PG_LOCKED_PBASE			/* Physical address of locked base memory */
	.long	CONFIG_PAGING_NLOCKED	/* Number of pages in the locked region */
	.long	MMU_L2_TEXTFLAGS		/* L2 MMU flags to use */
	.size	.Ltxtmap, . -.Ltxtmap

	.type	.Lptabspan, %object
.Lptabspan:
	.long	PG_L1_PGTABLE_PADDR		/* Physical address in the L1 table */
	.long	PG_L2_PGTABLE_PBASE		/* Physical address of the start of the L2 page table */
	.long	PG_PGTABLE_NPAGES		/* Total mapped page table pages */
	.long	PG_L2_PGTABLE_NPAGE1	/* The number of text pages in the first page table */
	.long	MMU_L1_PGTABFLAGS		/* L1 MMU flags to use */
	.size	.Lptabspan, . -.Lptabspan

	.type	.Lptabmap, %object
.Lptabmap:
	.long	PG_L2_PGTABLE_PADDR		/* Physical address in the L2 table */
	.long	PGTABLE_BASE_PADDR		/* Physical address of the page table memory */
	.long	PG_PGTABLE_NPAGES		/* Total mapped page table pages */
	.long	MMU_L2_PGTABFLAGS		/* L2 MMU flags to use */
	.size	.Lptabmap, . -.Lptabmap

	.size	__start, .-__start

/****************************************************************************
 * Name: .Lvstart
 ***************************************************************************/

/* The following is executed after the MMU has been enabled. This uses
 * absolute addresses; this is not position independent.
 */
	.align	5
	.local	.Lvstart
	.type	.Lvstart, %function

.Lvstart:

#ifndef CONFIG_IDENTITY_TEXTMAP
	/* Remove the temporary mapping (if one was made).  The following assumes
	 * that the total RAM size is > 1Mb and extends that initial mapping to
	 * cover additional RAM sections.
	 */

	ldr		r4, .LCvpgtable			/* r4=virtual page table base address */
	ldr		r3, .LCvtextbase		/* r0=virtual base address of .text section */
	mov		r0, #0					/* flags + base = 0 */
	str		r3, [r4, r3, lsr #18]	/* identity mapping */
#endif

	/* Populate the L1 table for the data region */

	adr		r0, .Ldataspan
	ldmia	r0, {r0, r1, r2, r3, r4}
	pg_l1span r0, r1, r2, r3, r4, r5

	/* Populate the L2 table for the data region */

	adr		r0, .Ldatamap
	ldmia	r0, {r0, r1, r2, r3}
	pg_l2map r0, r1, r2, r3, r4

#ifdef CONFIG_BOOT_RUNFROMFLASH
	/* Get R3 = Value of RAM L1 page table entry */

	ldr		r3, .LCprambase			/* r3=Aligned Nuttx RAM address (physical) */
	ldr		r1, .LCramflags			/* R1=.bss/.data section MMU flags */
	add		r3, r3, r1				/* r3=flags + base */

	/* Now setup the page tables for our normal mapped RAM region.
	 * We round NUTTX_RAM_VADDR down to the nearest megabyte boundary.
	 */

	add		r0, r4, #(NUTTX_RAM_VADDR & 0xfff00000) >> 18
	str		r3, [r0], #4

	/* Now map the remaining WR_NSECTIONS-1 sections of the RAM memory
	 * region.
	 */

	.rept	WR_NSECTIONS-1
	add		r3, r3, #SECTION_SIZE
	str		r3, [r0], #4
	.endr
#endif /* CONFIG_BOOT_RUNFROMFLASH */

	/* Initialize .bss and .data ONLY if .bss and .data lie in SRAM that is
	 * ready to use.  Other memory, such as SDRAM, must be initialized before
	 * it can be used.  arm_boot() will perform that memory initialization and
	 * .bss and .data can be initialized after arm_boot() returns.
	 */

	/* Set up the stack pointer and clear the frame pointer */

	ldr		sp, .Lstackpointer
	mov     fp, #0

#ifndef CONFIG_BOOT_SDRAM_DATA
	/* Initialize .bss and .data ONLY if .bss and .data lie in SRAM that is
	 * ready to use.  Other memory, such as SDRAM, must be initialized before
	 * it can be used.  arm_boot() will perform that memory initialization and
	 * .bss and .data can be initialized after arm_boot() returns.
	 */

	bl		arm_data_initialize
#endif

	/* Perform early C-level, platform-specific initialization.  Logic
	 * within arm_boot() must configure SDRAM and call arm_data_initialize().
	 */

	bl		arm_boot

#ifdef CONFIG_STACK_COLORATION
	/* Write a known value to the IDLE thread stack to support stack
	 * monitoring logic
	 */

	adr		r3, .Lstkinit
	ldmia	r3, {r0, r1, r2}	/* R0 = start of IDLE stack; R1 = Size of tack; R2 = coloration */

1:								/* Top of the loop */
	sub		r1, r1, #1			/* R1 = Number of words remaining */
	cmp		r1, #0				/* Check (nwords == 0) */
	str		r2, [r0], #4		/* Save stack color word, increment stack address */
	bne		1b					/* Bottom of the loop */
#endif

	/* Finally branch to the OS entry point */

	mov		lr, #0				/* LR = return address (none) */
	b		os_start			/* Branch to os_start */
	.size	.Lvstart, .-.Lvstart

/***************************************************************************
 * Name: arm_data_initialize
 ***************************************************************************/

	.global	arm_data_initialize
	.type	arm_data_initialize, #function

arm_data_initialize:

	/* Zero BSS */

	adr		r0, .Linitparms
	ldmia	r0, {r0, r1}

	mov     r2, #0
1:
	cmp		r0, r1				/* Clear up to _bss_end_ */
	strcc	r2, [r0],#4
	bcc		1b

#ifdef CONFIG_BOOT_RUNFROMFLASH
	/* If the .data section is in a separate, uninitialized address space,
	 * then we will also need to copy the initial values of of the .data
	 * section from the .text region into that .data region.  This would
	 * be the case if we are executing from FLASH and the .data section
	 * lies in a different physical address region OR if we are support
	 * on-demand paging and the .data section lies in a different virtual
	 * address region.
	 */

	adr		r3, .Ldatainit
	ldmia	r3, {r0, r1, r2}

2:
	ldr		r3, [r0], #4
	str		r3, [r1], #4
	cmp		r1, r2
	blt		2b
#endif

	/* And return to the caller */

	bx		lr
	.size	arm_data_initialize, . - arm_data_initialize

/***************************************************************************
 * Text-section constants
 ***************************************************************************/

	/* Text-section constants:
	 *
	 *   _sbss is the start of the BSS region (see ld.script)
	 *   _ebss is the end of the BSS regsion (see ld.script)
	 *
	 * The idle task stack usually starts at the end of BSS and is of size
	 * CONFIG_IDLETHREAD_STACKSIZE.  The heap continues from there until the
	 * end of memory.  See g_idle_topstack below.
	 *
	 * In the case where CONFIG_BOOT_SDRAM_DATA is defined, the IDLE stack is
	 * in ISRAM, but the heap is in SDRAM beginning at _ebss and extending
	 * to the end of SDRAM.
	 */

	.type	.Linitparms, %object
.Linitparms:
	.long	_sbss
	.long	_ebss
	.size	.Linitparms, . -.Linitparms

.Lstackpointer:
#ifdef CONFIG_BOOT_SDRAM_DATA
	.long	IDLE_STACK_VBASE+CONFIG_IDLETHREAD_STACKSIZE-4
#else
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE-4
#endif
	.size	.Lstackpointer, . -.Lstackpointer

	.type	.Ldataspan, %object
.Ldataspan:
	.long	PG_L1_DATA_VADDR		/* Virtual address in the L1 table */
	.long	PG_L2_DATA_PBASE		/* Physical address of the start of the L2 page table */
	.long	PG_DATA_NPAGES			/* Number of pages in the data region */
	.long	PG_L2_DATA_NPAGE1		/* The number of text pages in the first page table */
	.long	MMU_L1_DATAFLAGS		/* L1 MMU flags to use */
	.size	.Ldataspan, . -.Ldataspan

	.type	.Ldatamap, %object
.Ldatamap:
	.long	PG_L2_DATA_VADDR		/* Virtual address in the L2 table */
	.long	PG_DATA_PBASE			/* Physical address of data memory  */
	.long	PG_DATA_NPAGES			/* Number of pages in the data region */
	.long	MMU_L2_DATAFLAGS		/* L2 MMU flags to use */
	.size	.Ldatamap, . -.Ldatamap

	.type	.Ldatainit, %object
.Ldatainit:
	.long	_eronly					/* Where .data defaults are stored in FLASH */
	.long	_sdata					/* Where .data needs to reside in SDRAM */
	.long	_edata
	.size	.Ldatainit, . -.Ldatainit

#ifdef CONFIG_STACK_COLORATION
	.type	.Lstkinit, %object
.Lstkinit:
#ifdef CONFIG_BOOT_SDRAM_DATA
	.long	IDLE_STACK_VBASE		/* Beginning of the IDLE stack, then words of IDLE stack */
#else
	.long	_ebss					/* Beginning of the IDLE stack, then words of IDLE stack */
#endif
	.long	(CONFIG_IDLETHREAD_STACKSIZE >> 2)
	.long	STACK_COLOR				/* Stack coloration word */
	.size	.Lstkinit, . -.Lstkinit
#endif

/***************************************************************************
 * Data section variables
 ***************************************************************************/

	/* This global variable is unsigned long g_idle_topstack and is
	 * exported from here only because of its coupling to .Linitparms
	 * above.
	 */

	.section	.rodata, "a"
	.align	4
	.globl	g_idle_topstack
	.type	g_idle_topstack, object

g_idle_topstack:

#ifdef CONFIG_BOOT_SDRAM_DATA
	.long	IDLE_STACK_VBASE+CONFIG_IDLETHREAD_STACKSIZE
#else
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
#endif
	.size	g_idle_topstack, .-g_idle_topstack
	.end

/****************************************************************************
 * arch/arm/src/armv7-a/arm_head.S
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm.h"
#include "cp15.h"
#include "sctlr.h"
#include "mmu.h"
#include "chip.h"
#include "arm_internal.h"

#ifndef CONFIG_PAGING

	.file	"arm_head.S"

/****************************************************************************
 * Configuration
 ****************************************************************************/

/* Hard-coded options */

#undef CPU_ALIGNMENT_TRAP
#undef CPU_CACHE_ROUND_ROBIN
#undef CPU_DCACHE_DISABLE
#undef CPU_ICACHE_DISABLE
#undef CPU_AFE_ENABLE

/* There are three operational memory configurations:
 *
 * 1. We execute in place in FLASH (CONFIG_BOOT_RUNFROMFLASH=y).  In this case
 *    the boot logic must:
 *
 *    - Configure SDRAM (if present),
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

/* 2. We boot in FLASH but copy ourselves to SDRAM from better performance.
 *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=y).  In this case
 *    the boot logic must:
 *
 *    - Configure SDRAM (if present),
 *    - Copy ourself to DRAM (after mapping it), and
 *    - Clear .bss section (data should be fully initialized)
 *
 *   In this case, we assume that the logic within this file executes from FLASH.
 */

#elif defined(CONFIG_BOOT_COPYTORAM)
#  error "configuration not implemented"

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

/* 3. There is bootloader that copies us to SDRAM (but probably not to the beginning)
 *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=n). In this case SDRAM
 *    was initialized by the boot loader, and this boot logic must:
 *
 *    - Clear .bss section (data should be fully initialized)
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
	bl		arm_lowputc
	.endm
#else
	.macro	showprogress, code
	.endm
#endif

/****************************************************************************
 * Name: __start
 ****************************************************************************/

/* We assume the bootloader has already initialized most of the h/w for
 * us and that only leaves us having to do some os specific things
 * below.
 */

	.text
	.global	__start
	.type	__start, #function

__start:
#if defined(CONFIG_SMP) && CONFIG_SMP_NCPUS > 1
	/* Get cpuindex, cpu0 continue boot, others wait event from cpu0 */

	mrc		CP15_MPIDR(r0)
	and		r0, r0, #0x3
	cmp		r0, #0
	beq		__cpu0_start
	wfe
	cmp		r0, #1
	beq		__cpu1_start
#  if CONFIG_SMP_NCPUS > 2
	cmp		r0, #2
	beq		__cpu2_start
#  endif
#  if CONFIG_SMP_NCPUS > 3
	cmp		r0, #3
	beq		__cpu3_start
#  endif
#  if CONFIG_SMP_NCPUS > 4
	cmp		r0, #4
	beq		__cpu4_start
#  endif

__cpu0_start:
#endif

	/* Make sure that we are in SYS mode with IRQs and FIQs disabled */

	cpsid		if, #PSR_MODE_SYS

	/* The MMU and caches should be disabled */

	mrc		CP15_SCTLR(r0)
	bic		r0, r0, #(SCTLR_M | SCTLR_C)
	bic		r0, r0, #(SCTLR_I)
	mcr		CP15_SCTLR(r0)

	/* Clear the 16K level 1 page table */

	ldr		r5, .LCppgtable			/* r5=phys. page table */
#ifndef CONFIG_ARCH_ROMPGTABLE
	mov		r0, r5
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
	ldmia		r0, {r1, r2, r3}		/* Load the page table description */

	/* A single page is sufficient to map the page table */

	orr		r0, r1, r3			/* OR MMU flags into physical address */
	str		r0, [r5, r2, lsr #18]		/* Map using the virtual address as an index */
#endif

	/* Load information needed to map the .text region.  After the ldmia, we
	 * will have:
	 *
	 *   R1 = Aligned, physical address of the start of the .text region
	 *   R2 = Aligned, virtual address of the start of the .text region
	 *   R3 = MMU flags associated with the .txt region
	 *   R4 = The number of 1MB sections in the mapping
	 *   R5 = The physical address of the L1 page table (from above)
	 */

	adr		r0, .LCtextinfo			/* Address of text info */
	ldmia		r0, {r1, r2, r3, r4}		/* Load the text description */

#ifndef CONFIG_IDENTITY_TEXTMAP
	/* Create identity mapping for first MB of the .text section to support
	 * this start-up logic executing out of the physical address space.  This
	 * identity mapping will be removed by .Lvstart (see below).  Of course,
	 * we would only do this if the physical-virtual mapping is not already
	 * the identity mapping.
	 */

	orr		r0, r1, r3			/* OR MMU flags into physical address */
	str		r0, [r5, r1, lsr #18]		/* Identity mapping */
#endif

	/* Map the entire .text region.  We do this before enabling caches so
	 * we know that the data will be in place in the data cache.  We map the
	 * entire text region because we don't know which parts are needed for
	 * start-up.
	 *
	 * The page table base address is in R5.  Each 32-bit page table entry
	 * maps 1 MB of address space and is indexed by the lower 20 bits of
	 * the virtual address in R2
	 */

	add		r2, r5, r2, lsr #18		/* R2=Offset page table address */

	/* Now loop until each page table entry has been written for the .text
	 * region.
	 */

.Lpgtextloop:
	orr		r0, r1, r3			/* R0: OR MMU flags into physical address */
	subs		r4, r4, #1			/* R4: Decrement the section count */
	str		r0, [r2], #4			/* Save page table entry, increment page table address */
	add		r1, r1, #(1024*1024)		/* R1: Increment the physical address */
	bne		.Lpgtextloop			/* Loop while R4 is non-zero */

#if defined(CONFIG_BOOT_RUNFROMFLASH) && !defined(CONFIG_BOOT_SDRAM_DATA)
	/* If we are executing from FLASH, then we will need additional mappings for
	 * the primary RAM region that holds the .data, .bss, stack, and heap memory.
	 *
	 *   Here we expect to have:
	 *   r5 = Address of the base of the L1 table
	 *
	 * Load information needed to map the .text region.  After the ldmia, we
	 * will have:
	 *
	 *   R1 = Aligned, physical address of the start of the .text region
	 *   R2 = Aligned, virtual address of the start of the .text region
	 *   R3 = MMU flags associated with the .txt region
	 *   R4 = The number of 1MB sections in the mapping
	 *   R5 = The physical address of the L1 page table (from above)
	 */

	adr		r0, .LCraminfo			/* Address of primary RAM info */
	ldmia		r0, {r1, r2, r3, r4}		/* Load the primary RAM description */
	add		r2, r5, r2, lsr #18		/* R2=Offset page table address */

	/* Loop until each page table entry has been written for the primary RAM
	 * region.
	 */

.Lpgramloop:
	orr		r0, r1, r3			/* R0: OR MMU flags into physical address */
	subs		r4, r4, #1			/* R4: Decrement the section count */
	str		r0, [r2], #4			/* Save page table entry, increment page table address */
	add		r1, r1, #(1024*1024)		/* R1: Increment the physical address */
	bne		.Lpgramloop			/* Loop while R4 is non-zero */

#endif /* CONFIG_BOOT_RUNFROMFLASH && !CONFIG_BOOT_SDRAM_DATA */
#endif /* CONFIG_ARCH_ROMPGTABLE */

	/* The following logic will set up the ARMv7-A for normal operation.
	 *
	 * Here we expect to have:
	 *   r5 = Address of the base of the L1 table
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
#ifdef CONFIG_ARM_HAVE_MPCORE
	mcr		CP15_TLBIALLIS(r0)	/* Invalidate the entire unified TLB */
	mcr		CP15_BPIALLIS(r0)	/* Invalidate entire branch prediction array */
	mcr		CP15_ICIALLUIS(r0)	/* Invalidate I-cache */
#else
	mcr		CP15_TLBIALL(r0,c7)	/* Invalidate the entire unified TLB */
	mcr		CP15_TLBIALL(r0,c6)
	mcr		CP15_TLBIALL(r0,c5)
	mcr		CP15_BPIALL(r0)		/* Invalidate entire branch prediction array */
	mcr		CP15_ICIALLU(r0)	/* Invalidate I-cache */
#endif

	/* Load the page table address.
	 *
	 * NOTES:
	 * - Here we assume that the page table address is aligned to at least
	 *   least a 16KB boundary (bits 0-13 are zero).  No masking is provided
	 *   to protect against an unaligned page table address.
	 * - The ARMv7-A has two page table address registers, TTBR0 and 1.
	 *   Only TTBR0 is used in this implementation but both are initialized.
	 *
	 * Here we expect to have:
	 *   r0 = Zero
	 *   r5 = Address of the base of the L1 table
	 */

	orr		r1, r5, #(TTBR0_RGN_WBWA | TTBR0_IRGN0)	/* Select cache properties */
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
	 *   SCTLR_M    Bit 0:  Enable the MMU
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
	 *   SCTLR_EE   Bit 25: 0=Little endian (reset value).
	 *   SCTLR_TRE  Bit 28: No memory region remapping (reset value)
	 *   SCTLR_AFE  Bit 29: Full, legacy access permissions behavior (reset value).
	 *   SCTLR_TE   Bit 30: All exceptions handled in ARM state (reset value).
	 */

	bic		r0, r0, #(SCTLR_A  | SCTLR_C)
	bic		r0, r0, #(SCTLR_SW | SCTLR_I   | SCTLR_V   | SCTLR_RR | SCTLR_HA)
	bic		r0, r0, #(SCTLR_EE | SCTLR_TRE | SCTLR_AFE | SCTLR_TE)

#ifndef CONFIG_SMP
	/* Set bits to enable the MMU
	 *
	 *   SCTLR_M     Bit 0:  Enable the MMU
	 *   SCTLR_Z     Bit 11: Program flow prediction control always enabled on A5
	 */

	orr		r0, r0, #(SCTLR_M)
#endif

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

#if !defined(CPU_ICACHE_DISABLE) && !defined(CONFIG_SMP)
	/* Icache enable
	 *
	 *   SCTLR_I    Bit 12: ICache enable
	 */

	orr		r0, r0, #(SCTLR_I)
#endif

#ifdef CPU_ALIGNMENT_TRAP
	/* Alignment abort enable
	 *
	 *   SCTLR_A    Bit 1:  Strict alignment enabled
	 */

	orr		r0, r0, #(SCTLR_A)
#endif

#ifdef CONFIG_ENDIAN_BIG
	/* Big endian mode
	 *
	 *   SCTLR_EE       Bit 25: 1=Big endian.
	 */

	orr		r0, r0, #(SCTLR_EE)
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
	.rept		12				/* Cortex A8 wants lots of NOPs here */
	nop
	.endr

	/* And "jump" to .Lvstart in the newly mapped virtual address space */

	mov		pc, lr

/****************************************************************************
 * PC_Relative Data
 ****************************************************************************/

	/* The physical base address of the page table */

	.type	.LCppgtable, %object
.LCppgtable:
	.long	PGTABLE_BASE_PADDR			/* Physical start of page table */
	.size	.LCppgtable, . -.LCppgtable

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
	.long	MMU_MEMFLAGS				/* MMU flags for text section in RAM */
	.size	.LCptinfo, . -.LCptinfo
#endif

#ifndef CONFIG_ARCH_ROMPGTABLE
	/* Text region description.  The order of these fields must not change
	 * because the values are loaded using ldmia:
	 *
	 * 1) The aligned, physical base address of the .text section
	 * 2) The aligned, virtual base address of the .text section
	 * 3) The MMU flags to use with the .text space mapping
	 * 4) The number of 1MB sections in the .text region
	 *
	 * Values provided for NUTTX_TEXT_* must all be properly aligned to 1MB
	 * address boundaries
	 */

	.type	.LCtextinfo, %object
.LCtextinfo:
.LCptextbase:
	.long	NUTTX_TEXT_PADDR			/* Physical base address */
	.long	NUTTX_TEXT_VADDR			/* Virtual base address */
.LCtextflags:
#ifdef CONFIG_BOOT_RUNFROMFLASH
	.long	MMU_ROMFLAGS				/* MMU flags text section in FLASH/ROM */
#else
	.long	MMU_MEMFLAGS				/* MMU flags for text section in RAM */
#endif
	.long	(NUTTX_TEXT_SIZE >> 20)			/* Number of 1MB read-execute sections */
	.size	.LCtextinfo, . -.LCtextinfo

#ifdef CONFIG_BOOT_RUNFROMFLASH
	/* Primary RAM region description.  The order of these fields must not change
	 * because the values are loaded using ldmia:
	 *
	 * 1) The aligned, physical base address of the primary RAM section
	 * 2) The aligned, virtual base address of the primary RAM section
	 * 3) The MMU flags to use with the primary RAM space mapping
	 * 4) The number of 1MB sections in the primary RAM region
	 *
	 * Values provided for NUTTX_RAM_* must all be properly aligned to 1MB
	 * address boundaries
	 */

	.type	.LCraminfo, %object
.LCraminfo:
	.long	NUTTX_RAM_PADDR				/* Physical base address */
	.long	NUTTX_RAM_VADDR				/* Virtual base address */
	.long	MMU_MEMFLAGS				/* MMU flags for primary RAM section */
	.long	(NUTTX_RAM_SIZE >> 20)			/* Number of 1MB read-execute sections */
	.size	.LCraminfo, . -.LCraminfo

#endif /* CONFIG_BOOT_RUNFROMFLASH */
#endif /* CONFIG_ARCH_ROMPGTABLE */

	/* The virtual start address of the second phase boot logic */

	.type	.LCvstart, %object
.LCvstart:
	.long	.Lvstart
	.size	.LCvstart, . -.LCvstart

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

#if !defined(CONFIG_ARCH_ROMPGTABLE) && !defined(CONFIG_IDENTITY_TEXTMAP)
	/* Remove the temporary mapping (if one was made).  The following assumes
	 * that the total RAM size is > 1Mb and extends that initial mapping to
	 * cover additional RAM sections.
	 */

	ldr		r5, .LCvpgtable			/* r5=Virtual page table base address */
	ldr		r3, .LCptextbase		/* r0=Physical base address of .text section */
	mov		r0, #0				/* flags + base = 0 */
	str		r3, [r5, r3, lsr #18]		/* identity mapping */
#endif /* !CONFIG_ARCH_ROMPGTABLE && !CONFIG_IDENTITY_TEXTMAP */

	/* Set up the stack pointer and clear the frame pointer */

	ldr		sp, .Lstackpointer
	bic		sp, sp, #7			/* Get the stack pointer with 8-byte alignment */
	mov		fp, #0

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

	/* Finally branch to the OS entry point */

	mov		lr, #0				/* LR = return address (none) */
	b		nx_start			/* Branch to nx_start */
	.size	.Lvstart, .-.Lvstart

/***************************************************************************
 * Name: arm_data_initialize
 ***************************************************************************/

	.global	arm_data_initialize
	.weak	arm_data_initialize
	.type	arm_data_initialize, #function

arm_data_initialize:

	/* Zero BSS */

	adr		r0, .Linitparms
	ldmia		r0, {r0, r1}

	mov		r2, #0
1:
	cmp		r0, r1				/* Clear up to _bss_end_ */
	strcc		r2, [r0], #4
	bcc		1b

#ifdef CONFIG_BOOT_RUNFROMFLASH
	/* If the .data section is in a separate, uninitialized address space,
	 * then we will also need to copy the initial values of the .data
	 * section from the .text region into that .data region.  This would
	 * be the case if we are executing from FLASH and the .data section
	 * lies in a different physical address region OR if we are support
	 * on-demand paging and the .data section lies in a different virtual
	 * address region.
	 */

	adr		r3, .Ldatainit
	ldmia		r3, {r0, r1, r2}

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
	 *   _sbss is the start of the BSS region (see linker script)
	 *   _ebss is the end of the BSS region (see linker script)
	 *
	 * Typical Configuration:
	 * The idle task stack usually starts at the end of BSS and is of size
	 * CONFIG_IDLETHREAD_STACKSIZE.  The heap continues from there until the
	 * end of memory.  See g_idle_topstack below.
	 *
	 * In the case where CONFIG_BOOT_SDRAM_DATA is defined, the IDLE stack is
	 * in ISRAM, but the heap is in SDRAM beginning at _ebss and extending
	 * to the end of SDRAM.
	 */

#ifndef CONFIG_ARCH_ROMPGTABLE
	/* The virtual base address of the page table */

	.type	.LCvpgtable, %object
.LCvpgtable:
	.long	PGTABLE_BASE_VADDR			/* Virtual start of page table */
	.size	.LCvpgtable, . -.LCvpgtable

#endif /* CONFIG_ARCH_ROMPGTABLE */

	.type	.Linitparms, %object
.Linitparms:
	.long	_sbss
	.long	_ebss
	.size	.Linitparms, . -.Linitparms

.Lstackpointer:
#ifdef CONFIG_BOOT_SDRAM_DATA
	.long	IDLE_STACK_VBASE+CONFIG_IDLETHREAD_STACKSIZE
#elif defined(CONFIG_SMP)
	.long	_enoinit+CONFIG_IDLETHREAD_STACKSIZE
#else
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
#endif
	.size	.Lstackpointer, . -.Lstackpointer

#ifdef CONFIG_BOOT_RUNFROMFLASH
	.type	.Ldatainit, %object
.Ldatainit:
	.long	_eronly					/* Where .data defaults are stored in FLASH */
	.long	_sdata					/* Where .data needs to reside in SDRAM */
	.long	_edata
	.size	.Ldatainit, . -.Ldatainit
#endif

/***************************************************************************
 * Data section variables
 ***************************************************************************/

	/* This global variable is unsigned long g_idle_topstack and is
	 * exported from here only because of its coupling to .Lstackpointer
	 * above.
	 */

	.section	.rodata, "a"
	.align	4
	.globl	g_idle_topstack
	.type	g_idle_topstack, object

g_idle_topstack:

#ifdef CONFIG_BOOT_SDRAM_DATA
	.long	IDLE_STACK_VBASE+CONFIG_IDLETHREAD_STACKSIZE
#elif defined(CONFIG_SMP)
	.long	_enoinit+CONFIG_IDLETHREAD_STACKSIZE
#else
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
#endif
	.size	g_idle_topstack, .-g_idle_topstack
	.end
#endif

/****************************************************************************
 * arch/arm/src/arm/up_head.S
 *
 *   Copyright (C) 2007, 2009-2010 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm.h"
#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

/**********************************************************************************
 * Configuration
 **********************************************************************************/

#undef ALIGNMENT_TRAP
#undef CPU_DCACHE_WRITETHROUGH
#undef CPU_CACHE_ROUND_ROBIN
#undef CPU_DCACHE_DISABLE
#undef CPU_ICACHE_DISABLE

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
#  error "Configuration not implemented"
#  define CONFIGURE_SDRAM

/* 2. We boot in FLASH but copy ourselves to DRAM from better performance.
 *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=y).  In this case
 *    the boot logic must:
 *
 *    - Configure SDRAM, 
 *    - Copy ourself to DRAM (after mapping it), and
 *    - Clear .bss section
 */

#elif defined(CONFIG_BOOT_COPYTORAM)
#  error "configuration not implemented
#  define CONFIG_SDRAM

/* 3. There is bootloader that copies us to DRAM (but probably not to the beginning)
 *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=n). In this case the
 *    boot logic must:
 *
 *    - Clear .bss section
 */

#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* RX_NSECTIONS determines the number of 1Mb sections to map for the
 * Read/eXecute address region.  This is based on CONFIG_DRAM_SIZE.  For most
 * ARM9 architectures, CONFIG_DRAM_SIZE describes the size of installed SDRAM.
 * But for other architectures, this might refer to the size of FLASH or
 * SRAM regions. (bad choice of naming).
 */

#define RX_NSECTIONS ((CONFIG_DRAM_SIZE+0x000fffff) >> 20)

/* If CONFIG_PAGING is defined, then
 * - RX_NPAGES determines the number of pages of size PAGESIZE that
 *   are required to span the locked, Read/eXecute .text region.
 * - RW_NPAGES determines the number of pages of size PAGESIZE this
 *   are required to span the Read/Write data region.
 */

#ifdef CONFIG_PAGING
#  define RX_NPAGES  CONFIG_PAGING_LOCKEDPAGES
#endif

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/* Since the page table is closely related to the NuttX base address, we can
 * convert the page table base address to the base address of the section
 * containing both.
 */

	.macro	mksection, section, pgtable
	bic	\section, \pgtable, #0x000ff000
	.endm

/* This macro will modify r0, r1, r2 and r14 */

#ifdef CONFIG_DEBUG
	.macro	showprogress, code
	mov	r0, #\code
	bl	up_lowputc
	.endm
#else
	.macro	showprogress, code
	.endm
#endif

/****************************************************************************
 * Name: __start
 ****************************************************************************/

/* We assume the bootloader has already initialized most of the h/w for us
 * and that only leaves us having to do some os specific things below.
 */
	.text
	.global	__start
	.type	__start, #function
__start:
	/* Make sure that we are in SVC mode with all IRQs disabled */

	mov	r0, #(SVC_MODE | PSR_I_BIT | PSR_F_BIT)
	msr	cpsr_c, r0

	/* Initialize DRAM using a macro provided by board-specific logic */

#ifdef CONFIG_SDRAM
	config_sdram
#endif
	/* Clear the 16K level 1 page table */

	ldr	r4, .LCppgtable			/* r4=phys. page table */
#ifndef CONFIG_ARCH_ROMPGTABLE
	mov	r0, r4
	mov	r1, #0
	add	r2, r0, #PGTABLE_SIZE
.Lpgtableclear:
	str	r1, [r0], #4
	str	r1, [r0], #4
	str	r1, [r0], #4
	str	r1, [r0], #4
	teq	r0, r2
	bne	.Lpgtableclear

	/* Create identity mapping for first MB of the .text section to support
	 * this startup logic executing out of the physical address space.  This
	 * identity mapping will be removed by .Lvstart (see below).
	 */

	mksection r0, r4			/* r0=phys. base section */
	ldr	r1, .LCmmuflags			/* FLGS=MMU_MEMFLAGS */
	add	r3, r1, r0			/* r3=flags + base */
	str	r3, [r4, r0, lsr #18]		/* identity mapping */

	/* Create a virtual single section mapping for the first MB of the .text
	 * address space. Now, we have the first 1MB mapping to both phyical and
	 * virtual addresses.  The rest of the .text mapping will be completed in
	 * .Lvstart once we have moved the physical mapping out of the way.
	 */

	ldr	r2, .LCvpgtable			/* r2=virt. page table */
	mksection r0, r2			/* r0=virt. base section */
	str	r3, [r4, r0, lsr #18]		/* identity mapping */

	/* NOTE: No .data/.bss access should be attempted.  This temporary mapping
	 * can only be assumed to cover the initial .text region.
	 */

#endif /* CONFIG_ARCH_ROMPGTABLE */

	/* The following logic will set up the ARM920/ARM926 for normal operation */

	mov	r0, #0
	mcr	p15, 0, r0, c7, c7		/* Invalidate I,D caches */
	mcr	p15, 0, r0, c7, c10, 4		/* Drain write buffer */
	mcr	p15, 0, r0, c8, c7		/* Invalidate I,D TLBs */
	mcr	p15, 0, r4, c2, c0		/* Load page table pointer */

#ifdef CPU_DCACHE_WRITETHROUGH
	mov	r0, #4				/* Disable write-back on caches explicitly */
	mcr	p15, 7, r0, c15, c0, 0
#endif 

	/* Enable the MMU and caches
	 * lr = Resume at .Lvstart with the MMU enabled
	 */

	ldr	lr, .LCvstart			/* Abs. virtual address */

	mov	r0, #0x1f			/* Domains 0, 1 = client */
	mcr	p15, 0, r0, c3, c0		/* Load domain access register */
	mrc	p15, 0, r0, c1, c0		/* Get control register */
	
	/* Clear bits (see arm.h)
	 *
	 * CR_R - ROM MMU protection
	 * CR_F - Implementation defined
	 * CR_Z - Implementation defined
	 *
	 * CR_A - Alignment abort enable
	 * CR_C - Dcache enable
	 * CR_W - Write buffer enable
	 *
	 * CR_I - Icache enable
	 */

	bic	r0, r0, #(CR_R|CR_F|CR_Z)
	bic	r0, r0, #(CR_A|CR_C|CR_W)
	bic	r0, r0, #(CR_I)

	/* Set bits (see arm.h)
	 *
	 * CR_M - MMU enable
	 * CR_P - 32-bit exception handler
	 * CR_D - 32-bit data address range
	 */

	orr	r0, r0, #(CR_M|CR_P|CR_D)
	
	/* In most architectures, vectors are relocated to 0xffff0000.
	 * -- but not all
	 *
	 * CR_S - System MMU protection
	 * CR_V - Vectors relocated to 0xffff0000
	 */

#ifndef CONFIG_ARCH_LOWVECTORS
	orr	r0, r0, #(CR_S|CR_V)
#else
	orr	r0, r0, #(CR_S)
#endif
	/* CR_RR - Round Robin cache replacement */

#ifdef CPU_CACHE_ROUND_ROBIN
	orr	r0, r0, #(CR_RR)
#endif
	/* CR_C - Dcache enable */

#ifndef CPU_DCACHE_DISABLE
	orr	r0, r0, #(CR_C)
#endif
	/* CR_C - Dcache enable */

#ifndef CPU_ICACHE_DISABLE
	orr	r0, r0, #(CR_I)
#endif
	/* CR_A - Alignment abort enable */

#ifdef ALIGNMENT_TRAP
	orr	r0, r0, #(CR_A)
#endif
	mcr	p15, 0, r0, c1, c0, 0		/* write control reg */

	/* Get TMP=2 Processor ID register */

	mrc	p15, 0, r1, c0, c0, 0		/* read id reg */
	mov	r1, r1
	mov	r1, r1

	mov	pc, lr

/****************************************************************************
 * PC_Relative Data
 ****************************************************************************/

	/* These addresses are all virtual address */
	
	.type	.LCvstart, %object
.LCvstart:
	.long	.Lvstart

#ifndef CONFIG_ARCH_ROMPGTABLE
	.type	.LCmmuflags, %object
.LCmmuflags:
	.long	MMU_MEMFLAGS
#endif

	.type	.LCppagetable, %object
.LCppgtable:
	.long	PGTABLE_BASE_PADDR		/* Physical start of DRAM */

#ifndef CONFIG_ARCH_ROMPGTABLE
	.type	.LCvpagetable, %object
.LCvpgtable:
	.long	PGTABLE_BASE_VADDR		/* Virtual start of DRAM */
#endif
	.size	_start, .-_start

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

	/* Remove the temporary mapping.  The following assumes that the
	 * total RAM size is > 1Mb and extends that initial mapping to
	 * cover additinal RAM sections.
	 */

#ifndef CONFIG_ARCH_ROMPGTABLE
	ldr	r4, .LCvpgtable			/* r4=virtual page table */
	ldr	r1, .LCppgtable			/* r1=phys. page table */
	mksection r3, r1			/* r2=phys. base addr */
	mov	r0, #0				/* flags + base = 0 */
	str	r0, [r4, r3, lsr #18]		/* Undo identity mapping */

	/* Now setup the pagetables for our normal SDRAM mappings mapped region.
	 * We round NUTTX_START_VADDR down to the nearest megabyte boundary.
	 */

	ldr	r1, .LCmmuflags			/* FLGS=MMU_MEMFLAGS */
	add	r3, r3, r1			/* r3=flags + base */

	add	r0, r4, #(NUTTX_START_VADDR & 0xff000000) >> 18
	bic	r2, r3, #0x00f00000
	str	r2, [r0]

	add	r0, r0, #(NUTTX_START_VADDR & 0x00f00000) >> 18
	str	r3, [r0], #4

	/* Now map the remaining RX_NSECTIONS-1 sections of the executable
	 * memory region.
	 */

	.rept	RX_NSECTIONS-1
	add	r3, r3, #SECTION_SIZE
	str	r3, [r0], #4
	.endr

	/* If we are executing from RAM with a fixed page configuration, then
	 * we can assume that the above contiguous mapping included all of the
	 * .text, .data, .bss, heap, etc. But if we are executing from FLASH,
	 * then the RAM area is probably in a separate physical address region
	 * and will require a separate mapping.  Or, if we are supporting on-demand
	 * paging of the .text region, then the RAM-based .data/.bss/heap section
	 * will still probably be located in a separate (virtual) address region.
	 */

#if defined(CONFIG_BOOT_RUNFROMFLASH) || defined(CONFIG_PAGING)
# error "Logic not implemented"
#endif
#endif /* CONFIG_ARCH_ROMPGTABLE */

	/* Zero BSS and set up the stack pointer */

	adr	r0, .Linitparms
	ldmia	r0, {r0, r1, sp}

	/* Clear the frame pointer and .bss */

	mov     fp, #0

.Lbssinit:	
	cmp     r0, r1		       		/* Clear up to _bss_end_ */
	strcc	fp, [r0],#4
	bcc	.Lbssinit

	/* If the .data section is in a separate, unitialized address space,
	 * then we will also need to copy the initial values of of the .data
	 * section from the .text region into that .data region.  This would
	 * be the case if we are executing from FLASH and the .data section
	 * lies in a different physical address region OR if we are support
	 * on-demand paging and the .data section lies in a different virtual
	 * address region.
	 */

#if defined(CONFIG_BOOT_RUNFROMFLASH) || defined(CONFIG_PAGING)
# error "Logic not implemented"
#endif

	/* Perform early C-level, platform-specific initialization */

	bl	up_boot

	/* Finally branch to the OS entry point */

	mov	lr, #0
	b	os_start

	/* Variables:
	 *
	 *   _sbss is the start of the BSS region (see ld.script)
	 *   _ebss is the end of the BSS regsion (see ld.script)
	 *
	 * The idle task stack starts at the end of BSS and is of size
	 * CONFIG_IDLETHREAD_STACKSIZE.  The heap continues from there until the
	 * end of memory.  See g_heapbase below.
	 */

.Linitparms:
	.long	_sbss
	.long	_ebss
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE-4
	.size	.Lvstart, .-.Lvstart

	/* This global variable is unsigned long g_heapbase and is
	 * exported from here only because of its coupling to .Linitparms
	 * above.
	 */

	.data
	.align	4
	.globl	g_heapbase
	.type	g_heapbase, object
g_heapbase:
	.long	_ebss+CONFIG_IDLETHREAD_STACKSIZE
	.size	g_heapbase, .-g_heapbase
	.end


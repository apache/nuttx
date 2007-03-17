/********************************************************************
 * up_head.S
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ********************************************************************/

/********************************************************************
 * Included Files
 ********************************************************************/

#include <nuttx/config.h>
#include "up_internal.h"
#include "dm320.h"

/**************************************************************************
 * Conditional Compilation
 **************************************************************************/

#undef ALIGNMENT_TRAP
#undef CPU_DCACHE_WRITETHROUGH
#undef CPU_CACHE_ROUND_ROBIN
#undef CPU_DCACHE_DISABLE
#undef CPU_ICACHE_DISABLE

/********************************************************************
 * Definitions
 ********************************************************************/

/* The physical address of the beginning of SDRAM is provided by
 * CONFIG_DRAM_BASE.  The size of installed SDRAM is provided by
 * CONFIG_DRAM_SIZE.  The virtual address of SDRAM is provided by
 * DM320_SDRAM_VADDR.
 */

#define NSDRAM_SECTIONS (CONFIG_DRAM_SIZE >> 20)

/********************************************************************
 * Assembly Macros
 ********************************************************************/

/* Since the page table is closely related to the NuttX base
 * address, we can convert the page table base address to the
 * base address of the section containing both.
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

/********************************************************************
 * Name: __start
 ********************************************************************/

/* We assume the bootloader has already initialized most of the h/w for
 * us and that only leaves us having to do some os specific things
 * below.
 */
	.text
        .global __start
	.type	__start, #function
__start:
	/* Make sure that we are in SVC mode with all IRQs disabled */

	mov	r0, #(SVC_MODE | PSR_I_BIT | PSR_F_BIT)
	msr	cpsr_c, r0

	/* Clear the 16K level 1 page table */

	ldr	r4, .LCppgtable		/* r4=phys. page table */
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

	/* Create identity mapping for first MB section to support
	 * this startup logic executing out of the physical address
	 * space.  This identity mapping will be removed by .Lvstart
	 * (see below).
	 */

	mksection r0, r4			/* r0=phys. base section */
	ldr	r1, .LCmmuflags		/* FLGS=MMU_MEMFLAGS */
	add	r3, r1, r0		/* r3=flags + base */
	str	r3, [r4, r0, lsr #18]	/* identity mapping */

	/* Create a "normal" single section mapping for the first
	 * MB of memory.  Now, we have the first 1MB mapping to
	 * both phyical and virtual addresses.  The reset of the
	 * SDRAM mapping will be completed in .Lvstart once we have
	 * moved the physical mapping out of the way.
	 */

	ldr	r2, .LCvpgtable		/* r2=virt. page table */
	mksection r0, r2			/* r0=virt. base section */
	str	r3, [r4, r0, lsr #18]	/* identity mapping */

	/* The following logic will set up the ARM926 for normal operation */

	mov	r0, #0
	mcr	p15, 0, r0, c7, c7		/* Invalidate I,D caches */
	mcr	p15, 0, r0, c7, c10, 4	/* Drain write buffer */
	mcr	p15, 0, r0, c8, c7		/* Invalidate I,D TLBs */
	mcr	p15, 0, r4, c2, c0		/* Load page table pointer */

#ifdef CPU_DCACHE_WRITETHROUGH
	mov	r0, #4			/* Disable write-back on caches explicitly */
	mcr	p15, 7, r0, c15, c0, 0
#endif 

	/* Enable the MMU and caches
	 * lr = Resume at .Lvstart with the MMU enabled
	 */

	ldr	lr, .LCvstart			/* Abs. virtual address */

	mov	r0, #0x1f			/* Domains 0, 1 = client */
	mcr	p15, 0, r0, c3, c0		/* Load domain access register */
	mrc	p15, 0, r0, c1, c0		/* Get control register */
	
	/* Clear bits (see start.h) */

	bic	r0, r0, #(CR_R|CR_F|CR_Z)
	bic	r0, r0, #(CR_A|CR_C|CR_W)
	bic	r0, r0, #(CR_I)

	/* Set bits (see start.h) */

	orr	r0, r0, #(CR_M|CR_P|CR_D)
	orr	r0, r0, #(CR_S|CR_V)

#ifdef CPU_CACHE_ROUND_ROBIN
	orr	r0, r0, #(CR_RR)
#endif
#ifndef CPU_DCACHE_DISABLE
	orr	r0, r0, #(CR_C)
#endif
#ifndef CPU_ICACHE_DISABLE
	orr	r0, r0, #(CR_I)
#endif
#ifdef ALIGNMENT_TRAP
	orr	r0, r0, #(CR_A)
#endif
	mcr	p15, 0, r0, c1, c0, 0		/* write control reg */

	/* Get TMP=2 Processor ID register */

	mrc	p15, 0, r1, c0, c0, 0		/* read id reg */
	mov	r1, r1
	mov	r1, r1

	mov	pc, lr

/**************************************************************************
 * PC_Relative Data
 **************************************************************************/

	/* These addresses are all virtual address */
	
	.type	.LCvstart, %object
.LCvstart:
	.long	.Lvstart
	.type	.LCmmuflags, %object
.LCmmuflags:
	.long	MMU_MEMFLAGS
	.type	.LCppagetable, %object
.LCppgtable:
	.long	DM320_SDRAM_PADDR
	.type	.LCvpagetable, %object
.LCvpgtable:
	.long	DM320_SDRAM_VADDR
	.size	_start, .-_start

/**************************************************************************
 * Name: .Lvstart
 **************************************************************************/

/* The following is executed after the MMU has been enabled. This uses
 * absolute addresses; this is not position independent.
 */
	.align	5
	.local	.Lvstart
	.type	.Lvstart, %function
.Lvstart:

	/* Remove the temporary null mapping */

	ldr	r4, .LCvpgtable		/* r4=virtual page table */
	ldr	r1, .LCppgtable		/* r1=phys. page table */
	mksection r3, r1			/* r2=phys. base addr */
	mov	r0, #0			/* flags + base = 0 */
	str	r0, [r4, r3, lsr #18]	/* Undo identity mapping */

	/* Now setup the pagetables for our normal SDRAM mappings
	 * mapped region.  We round NUTTX_START_VADDR down to the
	 * nearest megabyte boundary.
	 */

	ldr	r1, .LCmmuflags		/* FLGS=MMU_MEMFLAGS */
	add	r3, r3, r1		/* r3=flags + base */

	add	r0, r4, #(NUTTX_START_VADDR & 0xff000000) >> 18
	bic	r2, r3, #0x00f00000
	str	r2, [r0]

	add	r0, r0, #(NUTTX_START_VADDR & 0x00f00000) >> 18
	str	r3, [r0], #4

	/* Now map the remaining NSDRAM_SECTIONS-1 SDRAM sections */

	.rept	NSDRAM_SECTIONS-1
	add	r3, r3, #SECTION_SIZE
	str	r3, [r0], #4
	.endr

	/* Zero BSS and set up the stack pointer */

	adr	r0, .Linitparms
	ldmia	r0, {r0, r1, sp}

	/* Clear the frame pointer and .bss */

	mov     fp, #0

.Lbssinit:	
	cmp     r0, r1		       /* Clear up to _bss_end_ */
	strcc	fp, [r0],#4
	bcc	.Lbssinit

	/* Perform early C-level initialization */

	bl	up_boot

	/* Perform early serial initialization */

#ifdef CONFIG_DEV_CONSOLE
	mov     fp, #0
	bl      up_earlyserialinit
#endif

	/* Finally branch to the OS entry point */

	mov	lr, #0
	b	os_start

	/* Variables:
	 * _sbss is the start of the BSS region (see ld.script)
	 * _ebss is the end of the BSS regsion (see ld.script)
	 * The idle task stack starts at the end of BSS and is
	 * of size CONFIG_PROC_STACK_SIZE.  The heap continues
	 * from there until the end of memory.  See g_heapbase
	 * below.
	 */

.Linitparms:
	.long	_sbss
	.long	_ebss
	.long	_ebss+CONFIG_PROC_STACK_SIZE-4
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
	.long	_ebss+CONFIG_PROC_STACK_SIZE
	.size	g_heapbase, .-g_heapbase
	.end


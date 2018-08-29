/****************************************************************************
 * arch/arm/src/armv7-r/arm_head.S
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include "arm.h"
#include "cp15.h"
#include "sctlr.h"

#include "up_internal.h"
#include "up_arch.h"

#include <arch/board/board.h>

/**********************************************************************************
 * Configuration
 **********************************************************************************/

/* Hard-coded options */

#undef CPU_ALIGNMENT_TRAP
#undef CPU_CACHE_ROUND_ROBIN
#undef CPU_DCACHE_DISABLE
#undef CPU_ICACHE_DISABLE
#define CPU_SCTLR_CCP15BEN 1
#define CPU_BACKGROUND_REGION 1
#undef CPU_DIV0_FAULT
#undef CPU_FAST_INTERRUPT
#undef CPU_IMPL_VECTORS
#undef CPU_NONMASKABLE_FIQ

/* There are three operational memory configurations:
 *
 * 1. We execute in place in FLASH (CONFIG_BOOT_RUNFROMFLASH=y).  In this case
 *    the boot logic must:
 *
 *    - Configure SDRAM (if present),
 *    - Initialize the .data section in RAM, and
 *    - Clear .bss section
 *
 * 2. We boot in FLASH but copy ourselves to SDRAM from better performance.
 *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=y).  In this case
 *    the boot logic must:
 *
 *    - Configure SDRAM (if present),
 *    - Copy ourself to RAM, and
 *    - Clear .bss section (data should be fully initialized)
 *
 *   In this case, we assume that the logic within this file executes from FLASH.
 *
 * 3. There is bootloader that copies us to SDRAM (CONFIG_BOOT_RUNFROMFLASH=n &&
 *    CONFIG_BOOT_COPYTORAM=n). In this case SDRAM was initialized by the boot
 *    loader, and this boot logic must:
 *
 *    - Clear .bss section (data should be fully initialized)
 */

/* Beginning (BOTTOM/BASE) and End+1 (TOP) of the IDLE stack.
 *
 * The IDLE stack is the stack that is used during intialization and,
 * eventually, becomes the stack of the IDLE task when initialization
 * is complete.
 *
 * REVISIT:  There are issues here in some configurations.  The stack
 * pointer is initialized very early in the boot sequence.  But in some
 * architectures the memory supporting the stack may not yet be
 * initialized (SDRAM, for example, would not be ready yet).  In that
 * case, ideally the IDLE stack should be in some other memory that does
 * not require initialization (such as internal SRAM)
 */

#ifndef IDLE_STACK_BASE
#  define IDLE_STACK_BASE _ebss
#endif

#define IDLE_STACK_TOP  IDLE_STACK_BASE+CONFIG_IDLETHREAD_STACKSIZE

/****************************************************************************
 * Global Symbols
 ****************************************************************************/

/* Imported symbols */

	.global	arm_boot			/* Branch to continue initialization in C */

	.global	_sbss				/* Start of .bss in RAM */
	.global	_ebss				/* End+1 of .bss in RAM */
#ifdef CONFIG_BOOT_RUNFROMFLASH
	.global	_eronly				/* Where .data defaults are stored in FLASH */
	.global	_sdata				/* Where .data needs to reside in SDRAM */
	.global	_edata
#endif
#ifdef CONFIG_ARCH_RAMFUNCS
	.global	_framfuncs			/* Where RAM functions are stored in FLASH */
	.global	_sramfuncs			/* Where RAM functions needs to reside in RAM */
	.global	_eramfuncs
#endif

/* Exported symbols */

	.global	__start				/* Power-up/Reset entry point */
	.global	arm_data_initialize	/* Perform C data initialization */
	.global	g_idle_topstack		/* Top of the initial/IDLE stack */

#ifdef CONFIG_ARCH_FPU
	.cpu	cortex-r4f
#else
	.cpu	cortex-r4
#endif
	.syntax	unified
	.file	"arm_head.S"

/***************************************************************************
 * .text
 ***************************************************************************/

	.text

/****************************************************************************
 * OS Entry Point
 ****************************************************************************/

/* We assume the bootloader has already initialized most of the h/w for
 * us and that only leaves us having to do some os specific things
 * below.
 */

	.type	__start, #function
__start:

	/* Make sure that we are in SVC mode with IRQs and FIQs disabled */

	mov		r0, #(PSR_MODE_SVC | PSR_I_BIT | PSR_F_BIT)
	msr		cpsr_c, r0

	/* Set up the stack pointer and clear the frame pointer. */

	ldr		sp, .Lstackpointer
	mov		fp, #0

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
	mcr		CP15_BPIALL(r0)		/* Invalidate entire branch prediction array */
	mcr		CP15_ICIALLU(r0)	/* Invalidate I-cache */
	mcr		CP15_DCIALLU(r0)	/* Invalidate D-cache */

	/* Configure the system control register (see sctrl.h) */

	mrc		CP15_SCTLR(r0)		/* Get control register */

	/* Clear bits to reset values.  This is only necessary in situations like, for
	 * example, we get here via a bootloader and the control register is in some
	 * unknown state.
	 *
	 *   SCTLR_M        Bit 0:  MPU enable bit
	 *   SCTLR_A        Bit 1:  Strict alignment disabled
	 *   SCTLR_C        Bit 2:  DCache disabled
	 *   SCTLR_CCP15BEN Bit 5:  CP15 barrier enable
	 *   SCTLR_B        Bit 7:  Should be zero on ARMv7R
	 *
	 *   SCTLR_SW       Bit 10: SWP/SWPB not enabled
	 *   SCTLR_I        Bit 12: ICache disabled
	 *   SCTLR_V        Bit 13: Assume low vectors
	 *   SCTLR_RR       Bit 14: Round-robin replacement strategy.
	 *
	 *   SCTLR_BR       Bit 17: Background Region bit
	 *   SCTLR_DZ       Bit 19: Divide by Zero fault enable bit
	 *   SCTLR_FI       Bit 21: Fast interrupts configuration enable bit
	 *   SCTLR_U        Bit 22: Unaligned access model (always one)
	 *
	 *   SCTLR_VE       Bit 24: Interrupt Vectors Enable bit
	 *   SCTLR_EE       Bit 25: 0=Little endian.
	 *   SCTLR_NMFI     Bit 27: Non-maskable FIQ (NMFI) support
	 *   SCTLR_TE       Bit 30: All exceptions handled in ARM state.
	 *   SCTLR_IE       Bit 31: Instruction endian-ness.
	 */

	/* Clear all configurable bits */

	bic		r0, r0, #(SCTLR_M | SCTLR_A  | SCTLR_C | SCTLR_CCP15BEN | SCTLR_B)
	bic		r0, r0, #(SCTLR_SW | SCTLR_I | SCTLR_V   | SCTLR_RR)
	bic		r0, r0, #(SCTLR_BR | SCTLR_DZ | SCTLR_FI | SCTLR_U)
	bic		r0, r0, #(SCTLR_VE | SCTLR_EE | SCTLR_NMFI | SCTLR_TE | SCTLR_IE)

	/* Set configured bits */

#ifdef CPU_ALIGNMENT_TRAP
	/* Alignment abort enable
	 *
	 *   SCTLR_A Bit 1:  Strict alignment enabled
	 */

	orr		r0, r0, #(SCTLR_A)
#endif

#ifndef CPU_DCACHE_DISABLE
	/* Dcache enable
	 *
	 *   SCTLR_C    Bit 2:  DCache enable
	 */

	orr		r0, r0, #(SCTLR_C)
#endif

#ifdef CPU_SCTLR_CCP15BEN
	/* Enable memory barriers
	 *
	 *  SCTLR_CCP15BEN Bit 5: CP15 barrier enable
	 */

	orr		r0, r0, #(SCTLR_CCP15BEN)
#endif

#ifndef CPU_ICACHE_DISABLE
	/* Icache enable
	 *
	 *   SCTLR_I    Bit 12: ICache enable
	 */

	orr		r0, r0, #(SCTLR_I)
#endif

#ifndef CONFIG_ARCH_LOWVECTORS
	/* Position vectors to 0xffff0000 if so configured.
	 *
	 *   SCTLR_V    Bit 13: High vectors
	 */

	orr		r0, r0, #(SCTLR_V)
#endif

#ifdef CPU_CACHE_ROUND_ROBIN
	/* Round Robin cache replacement
	 *
	 *   SCTLR_RR   Bit 14: Round-robin replacement strategy.
	 */

	orr		r0, r0, #(SCTLR_RR)
#endif

#ifdef CPU_BACKGROUND_REGION
	/* Allow PL1 access to back region when MPU is enabled
	 *
	 *  SCTLR_BR Bit 17: Background Region bit
	 */

	orr		r0, r0, #(SCTLR_BR)
#endif

#ifdef CPU_DIV0_FAULT
	/* Enable divide by zero faults
	 *
	 *   SCTLR_DZ Bit 19: Divide by Zero fault enable bit
	 */

	orr		r0, r0, #(SCTLR_DZ)
#endif

#ifdef CPU_FAST_INTERRUPT
	/* Fast interrupts configuration enable bit
	 *
	 *   SCTLR_FI Bit 21: Fast interrupts configuration enable bit
	 */

	orr		r0, r0, #(SCTLR_FI)
#endif

#ifdef CPU_IMPL_VECTORS
	/* Implementation defined interrupt vectors
	 *
	 *   SCTLR_VE Bit 24: Interrupt Vectors Enable bit
	 */

	orr		r0, r0, #(SCTLR_VE)
#endif

#ifdef CONFIG_ENDIAN_BIG
	/* Big endian mode
	 *
	 *   SCTLR_EE       Bit 25: 1=Big endian.
	 */

	orr		r0, r0, #(SCTLR_EE)
#endif

#ifdef CPU_NONMASKABLE_FIQ
	/* Non-maskable FIQ support
	 *
	 *   SCTLR_NMFI Bit 27: Non-maskable FIQ (NMFI) support
	 */

	orr		r0, r0, #(SCTLR_NMFI)
#endif

	/* Then write the configured control register */

	mcr		CP15_SCTLR(r0)			/* Write control reg */
	.rept	12						/* Some CPUs want want lots of NOPs here */
	nop
	.endr

#if 0
	/* Cortex-R/RF Errata Work-Around
	 *
	 * Errata 754269: Register corruption during a load-multiple instruction
	 *                at an exception vector.
	 * Workaround:    To workaround this erratum, set bit [7] of the
	 *                Auxiliary Control Register to disable out-of-order
	 *                completion for divide instructions. Code performance
	 *                may be reduced depending on how often divide
	 *                operations are used.
	 * NOTE:          The ARMv7-A/R Architecture reference many lists the bits
	 *                bits of the ACTLR as "implementation defined"
	 *
	 * REVISIT: I do not think this is needed because the NuttX vectors do not
	 * begin with an LDM instruction.
	 */

	mrc		CP15_ACTLR(r0)			/* Read Auxiliary Control register */
	orr		r0, r0, #(1 << 7)		/* Disable out-of-order completion */
	mcr		CP15_ACTLR(r0)			/* Write Auxiliary Control register */
#endif

#ifndef CONFIG_ARMV7R_MEMINIT
	/* Initialize .bss and .data ONLY if .bss and .data lie in RAM that is
	 * ready to use.  Other memory, such as SDRAM, must be initialized before
	 * it can be used.  arm_boot() will perform that memory initialization and
	 * .bss and .data can be initialized by arm_boot() by calling this
	 * arm_data_initialize() later.
	 */

	bl		arm_data_initialize
#endif

	/* Perform early C-level, platform-specific initialization.  Logic
	 * within arm_boot() must configure SDRAM and call arm_data_initialize()
	 * if CONFIG_ARMV7R_MEMINIT=y.
	 *
	 * This function does not return.  It must give control to os_start()
	 * at the completion of its initialization.
	 *
	 * Why not just call arm_boot() and branch to os_start() when it returns?
	 * If the stack pointer initialized above lies in SDRAM, then that may
	 * not be possible.  Also, in the special case of the TMS570, it may
	 * perform a destructive test, losing the pushed content of the stack.
	 */

	b		arm_boot

	/* .text Data */

.Lstackpointer:
	.long	IDLE_STACK_TOP
	.size	__start, .-__start

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
	strcc	r2, [r0], #4
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

#ifdef CONFIG_ARCH_RAMFUNCS
	/* Copy any necessary code sections from FLASH to RAM.  The correct
	 * destination in SRAM is given by _sramfuncs and _eramfuncs.  The
	 * temporary location is in flash after the data initialization code
	 * at _framfuncs
	 */

	adr		r3, .Lfuncinit
	ldmia	r3, {r0, r1, r2}

3:
	ldr		r3, [r0], #4
	str		r3, [r1], #4
	cmp		r1, r2
	blt		3b

#ifndef CPU_DCACHE_DISABLE
	/* Flush the copied RAM functions into physical RAM so that will
	 * be available when fetched into the I-Cache.
	 *
	 * Note that this is a branch, not a call and so will return
	 * directly to the caller without returning here.
	 */

	adr		r3, ..Lramfunc
	ldmia	r3, {r0, r1}
	ldr     r3, =arch_clean_dcache
	b		r3
#else
	/* Otherwise return to the caller */

	bx		lr
#endif
#else
	/* Return to the caller */

	bx		lr
#endif

	/* .text Data:
	 *
	 *   _sbss is the start of the BSS region (see linker script)
	 *   _ebss is the end of the BSS regsion (see linker script)
	 *
	 * Typical Configuration:
	 *   The idle task stack starts at the end of BSS and is of size
	 *   CONFIG_IDLETHREAD_STACKSIZE.  The heap continues from there
	 * until the end of memory.  See g_idle_topstack below.
	 */

	.type	.Linitparms, %object
.Linitparms:
	.long	_sbss
	.long	_ebss

#ifdef CONFIG_BOOT_RUNFROMFLASH
	.type	.Ldatainit, %object
.Ldatainit:
	.long	_eronly					/* Where .data defaults are stored in FLASH */
	.long	_sdata					/* Where .data needs to reside in SDRAM */
	.long	_edata
#endif

#ifdef CONFIG_ARCH_RAMFUNCS
	.type	.Lfuncinit, %object
.Lfuncinit:
	.long	_framfuncs				/* Where RAM functions are stored in FLASH */
.Lramfuncs:
	.long	_sramfuncs				/* Where RAM functions needs to reside in RAM */
	.long	_eramfuncs
#endif
	.size	arm_data_initialize, . - arm_data_initialize

/***************************************************************************
 * .rodata
 ***************************************************************************/

	/* This global variable is unsigned long g_idle_topstack and is
	 * exported from here only because of its coupling to .Lstackpointer
	 * above.
	 */

	.section	.rodata, "a"
	.align		4
	.type		g_idle_topstack, object
g_idle_topstack:
	.long		IDLE_STACK_TOP
	.size		g_idle_topstack, .-g_idle_topstack

	.end

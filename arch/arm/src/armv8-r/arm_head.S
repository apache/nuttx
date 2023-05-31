/****************************************************************************
 * arch/arm/src/armv8-r/arm_head.S
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
#include "cp15_cacheops.h"
#include "sctlr.h"
#include "arm_internal.h"

	.file	"arm_head.S"

/****************************************************************************
 * Configuration
 ****************************************************************************/

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
 *    - Copy ourself to DRAM, and
 *    - Clear .bss section (data should be fully initialized)
 *
 *   In this case, we assume that the logic within this file executes from FLASH.
 *
 * 3. There is bootloader that copies us to SDRAM (but probably not to the beginning)
 *    (CONFIG_BOOT_RUNFROMFLASH=n && CONFIG_BOOT_COPYTORAM=n). In this case SDRAM
 *    was initialized by the boot loader, and this boot logic must:
 *
 *    - Clear .bss section (data should be fully initialized)
 */

/* Beginning (BOTTOM/BASE) and End+1 (TOP) of the IDLE stack.
 *
 * The IDLE stack is the stack that is used during initialization and,
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
#  ifdef CONFIG_SMP
#    define IDLE_STACK_BASE _enoinit
#  else
#    define IDLE_STACK_BASE _ebss
#  endif
#endif

#define IDLE_STACK_TOP  IDLE_STACK_BASE+CONFIG_IDLETHREAD_STACKSIZE

/****************************************************************************
 * Global Symbols
 ****************************************************************************/

/* Imported symbols */

	.global	arm_boot			/* Branch to continue initialization in C */

	.global	_sbss				/* Start of .bss in RAM */
	.global	_ebss				/* End+1 of .bss in RAM */

	.global _hyp_vector_start
	.global _sys_vector_start
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

/****************************************************************************
 * Name: __start
 ****************************************************************************/

/* We assume the bootloader has already initialized most of the h/w for
 * us and that only leaves us having to do some os specific things
 * below.
 */

	.text
	.syntax	unified
	.arm
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

	/* Make sure that IRQs and FIQs are disabled */

	cpsid		if

	/* Set up the stack pointer and clear the frame pointer. */

	ldr		sp, .Lstackpointer
	mov		fp, #0

	/* Set Hyp/PL2 Vector table base register */
	ldr		r0, .Lhypvectorstart
	mcr		CP15_HVBAR(r0)

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
	mov     r0, CP15_CACHE_INVALIDATE
	bl		cp15_dcache_op_level
	isb

	bl		sctlr_initialize

	/* Initialize .bss and .data assumt that RAM that is ready to use. */
	bl		arm_data_initialize

    /* Platform hook for highest EL */
    bl      arm_el_init

    /* Move to PL1 SYS with all exceptions masked */
	mov		r0, #(PSR_MODE_SYS | PSR_I_BIT | PSR_F_BIT | PSR_A_BIT)
    msr		spsr_hyp, r0

    adr		r0, 1f
    msr		elr_hyp, r0
    eret

1:
	/* Set up the stack pointer and clear the frame pointer. */
	ldr		sp, .Lstackpointer
	mov		fp, #0

	/* Set PL1 Vector table base register */
	ldr		r0, .Lsysvectorstart
	mcr		CP15_VBAR(r0)

	bl		sctlr_initialize
	bl		arm_boot

	mov		lr, #0				/* LR = return address (none) */
	b		nx_start			/* Branch to nx_start */

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

#ifdef CONFIG_ARCH_RAMFUNCS
	/* Copy any necessary code sections from FLASH to RAM.  The correct
	 * destination in SRAM is given by _sramfuncs and _eramfuncs.  The
	 * temporary location is in flash after the data initialization code
	 * at _framfuncs
	 */

	adr		r3, .Lfuncinit
	ldmia		r3, {r0, r1, r2}

3:
	ldr		r3, [r0], #4
	str		r3, [r1], #4
	cmp		r1, r2
	blt		3b

#ifndef CONFIG_ARMV7R_DCACHE_DISABLE
	/* Flush the copied RAM functions into physical RAM so that will
	 * be available when fetched into the I-Cache.
	 *
	 * Note that this is a branch, not a call and so will return
	 * directly to the caller without returning here.
	 */

	adr		r3, ..Lramfunc
	ldmia		r3, {r0, r1}
	ldr		r3, =up_clean_dcache
	b		r3
#else
	/* Otherwise return to the caller */

	bx		lr
#endif
#else
	/* Return to the caller */

	bx		lr
#endif

/***************************************************************************
 * Name: arm_data_initialize
 ***************************************************************************/

	.global	sctlr_initialize
	.type	sctlr_initialize, #function

sctlr_initialize:
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
	 */

	/* Clear all configurable bits */

	bic		r0, r0, #(SCTLR_A  | SCTLR_C  | SCTLR_CCP15BEN | SCTLR_B)
	bic		r0, r0, #(SCTLR_SW | SCTLR_I  | SCTLR_V        | SCTLR_RR)
	bic		r0, r0, #(SCTLR_BR | SCTLR_DZ | SCTLR_FI)
	bic		r0, r0, #(SCTLR_VE | SCTLR_EE | SCTLR_NMFI     | SCTLR_TE)

#ifndef CONFIG_SMP
	/* Set bits to enable the MPU
	 *
	 *   SCTLR_M     Bit 0:  Enable the MPU
	 */

	orr		r0, r0, #(SCTLR_M)
#endif

	/* Set configured bits */

#ifdef CONFIG_ARMV8R_ALIGNMENT_TRAP
	/* Alignment abort enable
	 *
	 *   SCTLR_A Bit 1:  Strict alignment enabled
	 */

	orr		r0, r0, #(SCTLR_A)
#endif

#if !defined(CONFIG_ARMV8R_DCACHE_DISABLE) && !defined(CONFIG_SMP)
	/* Dcache enable
	 *
	 *   SCTLR_C    Bit 2:  DCache enable
	 */

	orr		r0, r0, #(SCTLR_C)
#endif

#ifdef CONFIG_ARMV8R_SCTLR_CCP15BEN
	/* Enable memory barriers
	 *
	 *  SCTLR_CCP15BEN Bit 5: CP15 barrier enable
	 */

	orr		r0, r0, #(SCTLR_CCP15BEN)
#endif

#if !defined(CONFIG_ARMV8R_ICACHE_DISABLE) && !defined(CONFIG_SMP)
	/* Icache enable
	 *
	 *   SCTLR_I    Bit 12: ICache enable
	 */

	orr		r0, r0, #(SCTLR_I)
#endif

#ifdef CONFIG_ARMV8R_CACHE_ROUND_ROBIN
	/* Round Robin cache replacement
	 *
	 *   SCTLR_RR   Bit 14: Round-robin replacement strategy.
	 */

	orr		r0, r0, #(SCTLR_RR)
#endif

#ifdef CONFIG_ARMV8R_BACKGROUND_REGION
	/* Allow PL1 access to back region when MPU is enabled
	 *
	 *  SCTLR_BR Bit 17: Background Region bit
	 */

	orr		r0, r0, #(SCTLR_BR)
#endif

#ifdef CONFIG_ARMV8R_DIV0_FAULT
	/* Enable divide by zero faults
	 *
	 *   SCTLR_DZ Bit 19: Divide by Zero fault enable bit
	 */

	orr		r0, r0, #(SCTLR_DZ)
#endif

#ifdef CONFIG_ARMV8R_FAST_INTERRUPT
	/* Fast interrupts configuration enable bit
	 *
	 *   SCTLR_FI Bit 21: Fast interrupts configuration enable bit
	 */

	orr		r0, r0, #(SCTLR_FI)
#endif

#ifdef CONFIG_ENDIAN_BIG
	/* Big endian mode
	 *
	 *   SCTLR_EE       Bit 25: 1=Big endian.
	 */

	orr		r0, r0, #(SCTLR_EE)
#endif

#ifdef CONFIG_ARMV8R_NONMASKABLE_FIQ
	/* Non-maskable FIQ support
	 *
	 *   SCTLR_NMFI Bit 27: Non-maskable FIQ (NMFI) support
	 */

	orr		r0, r0, #(SCTLR_NMFI)
#endif

	/* Then write the configured control register */

	mcr		CP15_SCTLR(r0)			/* Write control reg */
	isb
	.rept		12				/* Some CPUs want want lots of NOPs here */
	nop
	.endr

	/* Return to the caller */
	bx		lr

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
	 */

	.type	.Linitparms, %object
.Linitparms:
	.long	_sbss
	.long	_ebss

.Lhypvectorstart:
	.long	_hyp_vector_start
.Lsysvectorstart:
	.long	_sys_vector_start

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

	.long	IDLE_STACK_TOP
	.size	g_idle_topstack, .-g_idle_topstack
	.end

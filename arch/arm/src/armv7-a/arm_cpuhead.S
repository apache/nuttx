/****************************************************************************
 * arch/arm/src/armv7-a/arm_cpuhead.S
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

#include <arch/irq.h>

#include "arm.h"
#include "cp15.h"
#include "sctlr.h"
#include "mmu.h"
#include "smp.h"
#include "chip.h"
#include "arm_internal.h"

#ifdef CONFIG_SMP

	.file	"arm_cpuhead.S"

/****************************************************************************
 * Configuration
 ****************************************************************************/

/* Hard-coded options */

#undef CPU_ALIGNMENT_TRAP
#undef CPU_CACHE_ROUND_ROBIN
#undef CPU_DCACHE_DISABLE
#undef CPU_ICACHE_DISABLE
#undef CPU_AFE_ENABLE

/* Check for the identity mapping:  For this configuration, this would be
 * the case where the virtual beginning of RAM is the same as the physical
 * beginning of RAM.
 */

#if !defined(CONFIG_RAM_START) || !defined(CONFIG_RAM_VSTART)
#  error "CONFIG_RAM_START or CONFIG_RAM_VSTART is not defined"
#endif

#if CONFIG_RAM_START == CONFIG_RAM_VSTART
#  define CONFIG_IDENTITY_TEXTMAP 1
#endif

/****************************************************************************
 * .text
 ****************************************************************************/

	.text

/****************************************************************************
 * Name: __cpu[n]_start
 *
 * Description:
 *   Boot functions for each CPU (other than CPU0).  These functions set up
 *   the ARM operating mode, the initial stack, and configure co-processor
 *   registers.  At the end of the boot, arm_cpu_boot() is called.
 *
 *   These functions are provided by the common ARMv7-A logic.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Do not return.
 *
 ****************************************************************************/

#if CONFIG_SMP_NCPUS > 1
	.global	__cpu1_start
	.type	__cpu1_start, #function

__cpu1_start:
	/* Make sure that we are in SYS mode with IRQs and FIQs disabled */

	cpsid		if, #PSR_MODE_SYS

	/* Set up the stack pointer and the CPU index */

	ldr		sp, .Lcpu1_stackpointer
	sub		sp, sp, #XCPTCONTEXT_SIZE
	mov		r5, #1

	/* Then branch to the common startup logic (PC-relative) */

	b		.Lcpu_start

.Lcpu1_stackpointer:
	.long	.Lcpu1_stacktop
	.size	__cpu1_start, .-__cpu1_start

#if CONFIG_SMP_NCPUS > 2
	.global	__cpu2_start
	.type	__cpu2_start, #function

__cpu2_start:
	/* Make sure that we are in SYS mode with IRQs and FIQs disabled */

	cpsid		if, #PSR_MODE_SYS

	/* Set up the stack pointer and the CPU index */

	ldr		sp, .Lcpu2_stackpointer
	sub		sp, sp, #XCPTCONTEXT_SIZE
	mov		r5, #2

	/* Then branch to the common startup logic (PC-relative) */

	b		.Lcpu_start

.Lcpu2_stackpointer:
	.long	.Lcpu2_stacktop
	.size	__cpu2_start, .-__cpu2_start

#if CONFIG_SMP_NCPUS > 3
	.global	__cpu3_start
	.type	__cpu3_start, #function

__cpu3_start:
	/* Make sure that we are in SYS mode with IRQs and FIQs disabled */

	cpsid		if, #PSR_MODE_SYS

	/* Set up the stack pointer and the CPU index */

	ldr		sp, .Lcpu3_stackpointer
	sub		sp, sp, #XCPTCONTEXT_SIZE
	mov		r5, #3

	/* Then branch to the common startup logic (PC-relative) */

	b		.Lcpu_start

.Lcpu3_stackpointer:
	.long	.Lcpu3_stacktop
	.size	__cpu3_start, .-__cpu3_start

#if CONFIG_SMP_NCPUS > 4
#  error This logic needs to extended for CONFIG_SMP_NCPUS > 4

#endif /* CONFIG_SMP_NCPUS > 4 */
#endif /* CONFIG_SMP_NCPUS > 3 */
#endif /* CONFIG_SMP_NCPUS > 2 */
#endif /* CONFIG_SMP_NCPUS > 1 */

/****************************************************************************
 * Name: .Lcpu_start
 *
 * Description:
 *   Common CPUn startup logic (n > 0)
 *
 * On input:
 *   SP = Set to top of CPU IDLE stack (virtual)
 *   R5 = CPU number
 *
 ****************************************************************************/

	.type	.Lcpu_start, #function

.Lcpu_start:
	/* The MMU and caches should be disabled */

	mrc		CP15_SCTLR(r0)
	bic		r0, r0, #(SCTLR_M | SCTLR_C)
	bic		r0, r0, #(SCTLR_I)
	mcr		CP15_SCTLR(r0)

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
	 */

	ldr		r1, .LCppgtable		/* r1=phys. page table */
#ifdef CONFIG_ARCH_ADDRENV
	mov		r2, #PGTABLE_SIZE
	mla		r1, r2, r5, r1		/* page table of cpu1,2,3 */
#endif
	orr		r1, r1, #(TTBR0_RGN_WBWA | TTBR0_IRGN0)	/* Select cache properties */
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
	 * lr = Resume at .Lcpu_vstart with the MMU enabled
	 */

	ldr		lr, .LCcpu_vstart		/* Abs. virtual address */

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

#if 0 /* !defined(CPU_DCACHE_DISABLE) && !defined(CONFIG_SMP) */
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

	/* And "jump" to .Lcpu_vstart in the newly mapped virtual address space */

	mov		pc, lr

/****************************************************************************
 * PC_Relative Data
 ****************************************************************************/

	/* The physical base address of the page table */

	.type	.LCppgtable, %object
.LCppgtable:
	.long	PGTABLE_BASE_PADDR		/* Physical start of page table */
	.size	.LCppgtable, . -.LCppgtable

	/* The virtual start address of the second phase boot logic */

	.type	.LCcpu_vstart, %object
.LCcpu_vstart:
	.long	.Lcpu_vstart
	.size	.LCcpu_vstart, . -.LCcpu_vstart

	.size	.Lcpu_start, .-.Lcpu_start

/****************************************************************************
 * Name: .Lcpu_vstart
 *
 * Description:
 *   Continue initialization after the MMU has been enabled.
 *
 *   The following is executed after the MMU has been enabled. This uses
 *   absolute addresses; this is not position independent.
 *
 * On input:
 *   SP = Set to top of CPU IDLE stack (virtual)
 *   R5 = CPU number
 *
 ****************************************************************************/

	.align	8
	.globl	arm_cpu_boot
	.type	.Lcpu_vstart, %function

.Lcpu_vstart:

#ifdef CONFIG_STACK_COLORATION
	/* Write a known value to the IDLE thread stack to support stack
	 * monitoring logic
	 */

	adr		r3, .Lstkinit
	mov		r0, sp		/* R0 = end of IDLE stack */
	ldmia		r3, {r1, r2}	/* R1 = Size of stack; R2 = coloration */

1:							/* Top of the loop */
	sub		r1, r1, #1			/* R1 = Number of words remaining */
	cmp		r1, #0				/* Check (nwords == 0) */
	str		r2, [r0, #-4]!			/* Save stack color word, increment stack address */
	bne		1b				/* Bottom of the loop */
#endif

	/* Branch to continue C level CPU initialization */

	mov		fp, #0				/* Clear framepointer */
	mov		lr, #0				/* LR = return address (none) */
	mov		r0, r5				/* Input parameter = CPU index */
	b		arm_cpu_boot			/* Branch to C level CPU initialization */
	.size	.Lcpu_vstart, .-.Lcpu_vstart

/***************************************************************************
 * Text-section constants
 ***************************************************************************/

	/* Text-section constants: */

#ifdef CONFIG_STACK_COLORATION
	.type	.Lstkinit, %object
.Lstkinit:
	.long	SMP_STACK_WORDS - (XCPTCONTEXT_SIZE / 4)
	.long	STACK_COLOR				/* Stack coloration word */
	.size	.Lstkinit, . -.Lstkinit
#endif

/***************************************************************************
 * .noinit section data
 ***************************************************************************/

	.section	.noinit, "aw"

#if CONFIG_SMP_NCPUS > 1
	.align	8
	.globl	g_cpu1_idlestack
	.type	g_cpu1_idlestack, object

g_cpu1_idlestack:
	.space	SMP_STACK_SIZE
.Lcpu1_stacktop:
	.size	g_cpu1_idlestack, .Lcpu1_stacktop-g_cpu1_idlestack

#if CONFIG_SMP_NCPUS > 2
	.align	8
	.globl	g_cpu2_idlestack
	.type	g_cpu2_idlestack, object

g_cpu2_idlestack:
	.space	SMP_STACK_SIZE
.Lcpu2_stacktop:
	.size	g_cpu2_idlestack, .Lcpu2_stacktop-g_cpu2_idlestack

#if CONFIG_SMP_NCPUS > 3
	.align	8
	.globl	g_cpu3_idlestack
	.type	g_cpu3_idlestack, object

g_cpu3_idlestack:
	.space	SMP_STACK_SIZE
.Lcpu3_stacktop:
	.size	g_cpu3_idlestack, .Lcpu3_stacktop-g_cpu3_idlestack

#if CONFIG_SMP_NCPUS > 4
#  error This logic needs to extended for CONFIG_SMP_NCPUS > 4

#endif /* CONFIG_SMP_NCPUS > 4 */
#endif /* CONFIG_SMP_NCPUS > 3 */
#endif /* CONFIG_SMP_NCPUS > 2 */
#endif /* CONFIG_SMP_NCPUS > 1 */
#endif /* CONFIG_SMP */
	.end

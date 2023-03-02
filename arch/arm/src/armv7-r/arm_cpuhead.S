/****************************************************************************
 * arch/arm/src/armv7-r/arm_cpuhead.S
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
#include "smp.h"
#include "chip.h"
#include "arm_internal.h"

#ifdef CONFIG_SMP

	.file	"arm_cpuhead.S"

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

/****************************************************************************
 * .text
 ****************************************************************************/

	.text
	.syntax	unified
	.arm

/****************************************************************************
 * Name: __cpu[n]_start
 *
 * Description:
 *   Boot functions for each CPU (other than CPU0).  These functions set up
 *   the ARM operating mode, the initial stack, and configure co-processor
 *   registers.  At the end of the boot, arm_cpu_boot() is called.
 *
 *   These functions are provided by the common ARMv7-R logic.
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
 *   SP = Set to top of CPU IDLE stack
 *   R5 = CPU number
 *
 ****************************************************************************/

	.type	.Lcpu_start, #function

.Lcpu_start:
	/* The MPU and caches should be disabled */

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
	mcr		CP15_BPIALL(r0)		/* Invalidate entire branch prediction array */
	mcr		CP15_ICIALLU(r0)	/* Invalidate I-cache */
	mcr		CP15_DCIALLU(r0)	/* Invalidate D-cache */
	isb

	/* Set lr = Resume at .Lcpu_vstart with the MMU enabled */

	ldr		lr, .LCcpu_vstart		/* Abs. address */

	/* Configure the system control register (see sctrl.h) */

	mrc		CP15_SCTLR(r0)			/* Get control register */

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

	bic		r0, r0, #(SCTLR_A  | SCTLR_C  | SCTLR_CCP15BEN | SCTLR_B)
	bic		r0, r0, #(SCTLR_SW | SCTLR_I  | SCTLR_V        | SCTLR_RR)
	bic		r0, r0, #(SCTLR_BR | SCTLR_DZ | SCTLR_FI)
	bic		r0, r0, #(SCTLR_VE | SCTLR_EE | SCTLR_NMFI     | SCTLR_TE)

	/* Set bits to enable the MPU
	 *
	 *   SCTLR_M     Bit 0:  Enable the MPU
	 */

	orr		r0, r0, #(SCTLR_M)

	/* Set configured bits */

#ifdef CONFIG_ARMV7R_ALIGNMENT_TRAP
	/* Alignment abort enable
	 *
	 *   SCTLR_A Bit 1:  Strict alignment enabled
	 */

	orr		r0, r0, #(SCTLR_A)
#endif

#ifdef CONFIG_ARMV7R_SCTLR_CCP15BEN
	/* Enable memory barriers
	 *
	 *  SCTLR_CCP15BEN Bit 5: CP15 barrier enable
	 */

	orr		r0, r0, #(SCTLR_CCP15BEN)
#endif

#ifndef CONFIG_ARCH_LOWVECTORS
	/* Position vectors to 0xffff0000 if so configured.
	 *
	 *   SCTLR_V    Bit 13: High vectors
	 */

	orr		r0, r0, #(SCTLR_V)
#endif

#ifdef CONFIG_ARMV7R_CACHE_ROUND_ROBIN
	/* Round Robin cache replacement
	 *
	 *   SCTLR_RR   Bit 14: Round-robin replacement strategy.
	 */

	orr		r0, r0, #(SCTLR_RR)
#endif

#ifdef CONFIG_ARMV7R_BACKGROUND_REGION
	/* Allow PL1 access to back region when MPU is enabled
	 *
	 *  SCTLR_BR Bit 17: Background Region bit
	 */

	orr		r0, r0, #(SCTLR_BR)
#endif

#ifdef CONFIG_ARMV7R_DIV0_FAULT
	/* Enable divide by zero faults
	 *
	 *   SCTLR_DZ Bit 19: Divide by Zero fault enable bit
	 */

	orr		r0, r0, #(SCTLR_DZ)
#endif

#ifdef CONFIG_ARMV7R_FAST_INTERRUPT
	/* Fast interrupts configuration enable bit
	 *
	 *   SCTLR_FI Bit 21: Fast interrupts configuration enable bit
	 */

	orr		r0, r0, #(SCTLR_FI)
#endif

#ifdef CONFIG_ARMV7R_IMPL_VECTORS
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

#ifdef CONFIG_ARMV7R_NONMASKABLE_FIQ
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

	/* And "jump" to .Lcpu_vstart in the newly mapped virtual address space */

	mov		pc, lr

/****************************************************************************
 * PC_Relative Data
 ****************************************************************************/

	/* The start address of the second phase boot logic */

	.type	.LCcpu_vstart, %object
.LCcpu_vstart:
	.long	.Lcpu_vstart
	.size	.LCcpu_vstart, . -.LCcpu_vstart

	.size	.Lcpu_start, .-.Lcpu_start

/****************************************************************************
 * Name: .Lcpu_vstart
 *
 * Description:
 *   Continue initialization after the MPU has been enabled.
 *
 *   The following is executed after the MPU has been enabled. This uses
 *   absolute addresses; this is not position independent.
 *
 * On input:
 *   SP = Set to top of CPU IDLE stack
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

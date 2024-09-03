/****************************************************************************
 * arch/arm/src/armv8-r/arm_cpuhead.S
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
#include "cp15_cacheops.h"
#include "sctlr.h"
#include "smp.h"
#include "chip.h"
#include "arm_internal.h"

#if defined(CONFIG_SMP) || defined(CONFIG_BMP)

	.file	"arm_cpuhead.S"

/****************************************************************************
 * Global Symbols
 ****************************************************************************/

/* Imported symbols */

	.global _hyp_vector_start
	.global _sys_vector_start

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
 *   These functions are provided by the common ARMv7-A logic.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Do not return.
 *
 ****************************************************************************/

#if CONFIG_NR_CPUS > 1
	.global	__cpu1_start
	.type	__cpu1_start, #function

__cpu1_start:
	/* Make sure that we are in SYS mode with IRQs and FIQs disabled */

	cpsid		if

	/* Set up the stack pointer and the CPU index */

	ldr		sp, .Lcpu1_stackpointer
	mov		fp, #0

	/* Then branch to the common startup logic (PC-relative) */

	b		.Lcpu_start

.Lcpu1_stackpointer:
	.long	.Lcpu1_stacktop
	.size	__cpu1_start, .-__cpu1_start

#if CONFIG_NR_CPUS > 2
	.global	__cpu2_start
	.type	__cpu2_start, #function

__cpu2_start:
	/* Make sure that we are in SYS mode with IRQs and FIQs disabled */

	cpsid		if

	/* Set up the stack pointer and the CPU index */

	ldr		sp, .Lcpu2_stackpointer
	mov		fp, #0

	/* Then branch to the common startup logic (PC-relative) */

	b		.Lcpu_start

.Lcpu2_stackpointer:
	.long	.Lcpu2_stacktop
	.size	__cpu2_start, .-__cpu2_start

#if CONFIG_NR_CPUS > 3
	.global	__cpu3_start
	.type	__cpu3_start, #function

__cpu3_start:
	/* Make sure that we are in SYS mode with IRQs and FIQs disabled */

	cpsid		if

	/* Set up the stack pointer and the CPU index */

	ldr		sp, .Lcpu3_stackpointer
	mov		fp, #0

	/* Then branch to the common startup logic (PC-relative) */

	b		.Lcpu_start

.Lcpu3_stackpointer:
	.long	.Lcpu3_stacktop
	.size	__cpu3_start, .-__cpu3_start

#if CONFIG_NR_CPUS > 4
#  error This logic needs to extended for CONFIG_NR_CPUS > 4

#endif /* CONFIG_NR_CPUS > 4 */
#endif /* CONFIG_NR_CPUS > 3 */
#endif /* CONFIG_NR_CPUS > 2 */
#endif /* CONFIG_NR_CPUS > 1 */

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
	mov		r1, CP15_CACHE_INVALIDATE
	bl		cp15_dcache_op_level
	isb

	bl		hsctlr_initialize  /* Init Hyp system control register */

	ldr		r0, =HACTLR_INIT
	mcr		CP15_HACTLR(r0)  /* Enable EL1 access all IMP DEFINED registers */

	/* Initialize .bss and .data assumt that RAM that is ready to use. */
	bl		arm_data_initialize

	/* Platform hook for highest EL */
	bl		arm_el_init

    /* Move to PL1 SYS with all exceptions masked */
	mov		r0, #(PSR_MODE_SYS | PSR_I_BIT | PSR_F_BIT | PSR_A_BIT)
	msr		spsr_hyp, r0

	adr		r0, 1f
	msr		elr_hyp, r0
	dsb
	isb
	eret

1:
	mrc		CP15_MPIDR(r0)
	and		r0, r0, #0x3
#if CONFIG_NR_CPUS > 1
	cmp		r0, #1
	beq		__cpu1_stack
#endif
#if CONFIG_NR_CPUS > 2
	cmp		r0, #2
	beq		__cpu2_stack
#endif
#if CONFIG_NR_CPUS > 3
	cmp		r0, #3
	beq		__cpu3_stack
#endif

#if CONFIG_NR_CPUS > 1
__cpu1_stack:

	/* Set up the stack pointer and clear the frame pointer. */
	ldr		sp, .Lcpu1_stackpointer
  b 2f
#endif
#if CONFIG_NR_CPUS > 2
__cpu2_stack:

	/* Set up the stack pointer and clear the frame pointer. */
	ldr		sp, .Lcpu2_stackpointer
  b 2f
#endif
__cpu3_stack:

#if CONFIG_NR_CPUS > 3
	/* Set up the stack pointer and clear the frame pointer. */
	ldr		sp, .Lcpu3_stackpointer
#endif
2:
	mov		fp, #0

	/* Set PL1 Vector table base register */
	ldr		r0, .Lsysvectorstart
	mcr		CP15_VBAR(r0)

	bl		sctlr_initialize

#ifdef CONFIG_ARCH_FPU
	bl		arm_fpuconfig
#endif

	bl		arm_boot

	mov		lr, #0				/* LR = return address (none) */
	b		nx_start			/* Branch to nx_start */

/****************************************************************************
 * PC_Relative Data
 ****************************************************************************/

	.size	.Lcpu_start, .-.Lcpu_start

.Lhypvectorstart:
	.long	_hyp_vector_start
.Lsysvectorstart:
	.long	_sys_vector_start

/***************************************************************************
 * Text-section constants
 ***************************************************************************/

/***************************************************************************
 * .noinit section data
 ***************************************************************************/

	.section	.noinit, "aw"

#if CONFIG_NR_CPUS > 1
	.align	8
	.globl	g_cpu1_idlestack
	.type	g_cpu1_idlestack, object

g_cpu1_idlestack:
	.space	SMP_STACK_SIZE
.Lcpu1_stacktop:
	.size	g_cpu1_idlestack, .Lcpu1_stacktop-g_cpu1_idlestack

#if CONFIG_NR_CPUS > 2
	.align	8
	.globl	g_cpu2_idlestack
	.type	g_cpu2_idlestack, object

g_cpu2_idlestack:
	.space	SMP_STACK_SIZE
.Lcpu2_stacktop:
	.size	g_cpu2_idlestack, .Lcpu2_stacktop-g_cpu2_idlestack

#if CONFIG_NR_CPUS > 3
	.align	8
	.globl	g_cpu3_idlestack
	.type	g_cpu3_idlestack, object

g_cpu3_idlestack:
	.space	SMP_STACK_SIZE
.Lcpu3_stacktop:
	.size	g_cpu3_idlestack, .Lcpu3_stacktop-g_cpu3_idlestack

#if CONFIG_NR_CPUS > 4
#  error This logic needs to extended for CONFIG_NR_CPUS > 4

#endif /* CONFIG_NR_CPUS > 4 */
#endif /* CONFIG_NR_CPUS > 3 */
#endif /* CONFIG_NR_CPUS > 2 */
#endif /* CONFIG_NR_CPUS > 1 */
#endif /* CONFIG_SMP */
	.end

/****************************************************************************
 * arch/arm/src/arm/arm_vectors.S
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
#include <nuttx/irq.h>

#include "arm.h"

	.file	"arm_vectors.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

	.text

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_vectorirq
 *
 * Description:
 *   Interrupt exception. Entered in IRQ mode with spsr = SVC CPSR, lr = SVC PC
 *
 ****************************************************************************/

	.globl	arm_decodeirq
	.globl	arm_vectorirq
	.type	arm_vectorirq, %function

arm_vectorirq:
	/* On entry, we are in IRQ mode.  We are free to use the IRQ mode r13
	 * and r14.
	 */

	mov	r13, #(PSR_MODE_SVC | PSR_I_BIT)
	msr	cpsr_c, r13		/* Switch to SVC mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SVC mode regs */

	mov	r0, #(PSR_MODE_IRQ | PSR_I_BIT)
	msr	cpsr_c, r0			/* Switch back IRQ mode */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	sub	r3, lr, #4
	mrs	r4, spsr

	/* Then switch back to SVC mode */

	orr	r0, r0, #(PSR_MODE_SVC | PSR_I_BIT)
	msr	cpsr_c, r0

	/* Get the correct values of SVC r13(sp) and r14(lr) in r1 and r2 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

	/* Then call the IRQ handler with interrupts disabled. */

	mov	fp, #0			/* Init frame pointer */
	mov	r0, sp			/* Get r0=xcp */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	/* Call arm_decodeirq() on the interrupt stack */

	ldr	sp, .Lirqstacktop	/* SP = interrupt stack top */
	str	r0, [sp, #-4]!		/* Save the xcp address at SP-4 then update SP */
	bl	arm_decodeirq		/* Call the handler */
	ldr	sp, [sp]		/* Restore the user stack pointer */
#else
	/* Call arm_decodeirq() on the user stack */

	bl	arm_decodeirq		/* Call the handler */
#endif

	/* Restore the CPSR, SVC mode registers and return */

	ldr	r0, [sp, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr	spsr_cxsf, r0		/* Set the return mode SPSR */
	ldmia	sp, {r0-r15}^		/* Return */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
.Lirqstacktop:
	.word	g_intstacktop
#endif
	.size	arm_vectorirq, . - arm_vectorirq

	.align	5

/****************************************************************************
 * Function: arm_vectorsvc
 *
 * Description:
 *   SVC interrupt. We enter the SVC in SVC mode.
 *
 ****************************************************************************/

	.globl	arm_syscall
	.globl	arm_vectorsvc
	.type	arm_vectorsvc, %function

arm_vectorsvc:

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SVC mode regs */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	mov	r3, r14			/* Save r14 as the PC as well */
	mrs	r4, spsr		/* Get the saved CPSR */

	/* Get the correct values of SVC r13(sp) and r14(lr) in r1 and r2 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

	/* Then call the SVC handler with interrupts disabled.
	 * void arm_syscall(struct xcptcontext *xcp)
	 */

	mov	fp, #0			/* Init frame pointer */
	mov	r0, sp			/* Get r0=xcp */
	bl	arm_syscall		/* Call the handler */

	/* Restore the CPSR, SVC mode registers and return */

	ldr	r0, [sp, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr	spsr_cxsf, r0		/* Set the return mode SPSR */
	ldmia	sp, {r0-r15}^		/* Return */
	.size	arm_vectorsvc, . - arm_vectorsvc

	.align	5

/****************************************************************************
 * Name: arm_vectordata
 *
 * Description:
 *   This is the data abort exception dispatcher. The ARM data abort exception occurs
 *   when a memory fault is detected during a data transfer.  This handler saves the
 *   current processor state and gives control to data abort handler.  This function
 *   is entered in ABORT mode with spsr = SVC CPSR, lr = SVC PC
 *
 ****************************************************************************/

	.globl	arm_dataabort
	.globl	arm_vectordata
	.type	arm_vectordata, %function

arm_vectordata:
	/* On entry we are free to use the ABORT mode registers
	 * r13 and r14
	 */

	mov	r13, #(PSR_MODE_SVC | PSR_I_BIT)
	msr	cpsr_c, r13		/* Switch to SVC mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SVC mode regs */

	mov	r0, #(PSR_MODE_ABT | PSR_I_BIT)
	msr	cpsr_c, r0		/* Switch back ABT mode */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	sub	r3, lr, #8
	mrs	r4, spsr

	/* Then switch back to SVC mode */

	mov	r0, #(PSR_MODE_SVC | PSR_I_BIT)
	msr	cpsr_c, r0

	/* Get the correct values of SVC r13(sp) and r14(lr) in r1 and r2 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

	/* Then call the data abort handler with interrupts disabled.
	 * void arm_dataabort(struct xcptcontext *xcp)
	 */

	mov	fp, #0			/* Init frame pointer */
	mov	r0, sp			/* Get r0=xcp */
#ifdef CONFIG_PAGING
	mrc	p15, 0, r1, c6, c0, 0	/* Get R1=FAR */
	mrc	p15, 0, r2, c5, c0, 0	/* Get r2=FSR */
#endif
	bl	arm_dataabort		/* Call the handler */

	/* Restore the CPSR, SVC mode registers and return */

	ldr	r0, [sp, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr	spsr_cxsf, r0		/* Set the return mode SPSR */
	ldmia	sp, {r0-r15}^		/* Return */
	.size	arm_vectordata, . - arm_vectordata

	.align	5

/****************************************************************************
 * Name: arm_vectorprefetch
 *
 * Description:
 *   This is the prefetch abort exception dispatcher. The ARM prefetch abort exception
 *   occurs when a memory fault is detected during an an instruction fetch.  This
 *   handler saves the current processor state and gives control to prefetch abort
 *   handler.  This function is entered in ABT mode with spsr = SVC CPSR, lr = SVC PC.
 *
 ****************************************************************************/

	.globl	arm_prefetchabort
	.globl	arm_vectorprefetch
	.type	arm_vectorprefetch, %function

arm_vectorprefetch:
	/* On entry we are free to use the ABORT mode registers
	 * r13 and r14
	 */

	mov	r13, #(PSR_MODE_SVC | PSR_I_BIT)
	msr	cpsr_c, r13		/* Switch to SVC mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SVC mode regs */

	mov	r0, #(PSR_MODE_ABT | PSR_I_BIT)
	msr	cpsr_c, r0		/* Switch back ABT mode */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	sub	r3, lr, #4
	mrs	r4, spsr

	/* Then switch back to SVC mode */

	mov	r0, #(PSR_MODE_SVC | PSR_I_BIT)
	msr	cpsr_c, r0

	/* Get the correct values of SVC r13(sp) and r14(lr) in r1 and r2 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

	/* Then call the prefetch abort handler with interrupts disabled.
	 * void arm_prefetchabort(struct xcptcontext *xcp)
	 */

	mov	fp, #0			/* Init frame pointer */
	mov	r0, sp			/* Get r0=xcp */
	bl	arm_prefetchabort	/* Call the handler */

	/* Restore the CPSR, SVC mode registers and return */

	ldr	r0, [sp, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr	spsr_cxsf, r0		/* Set the return mode SPSR */
	ldmia	sp, {r0-r15}^		/* Return */
	.size	arm_vectorprefetch, . - arm_vectorprefetch

	.align	5

/****************************************************************************
 * Name: arm_vectorundefinsn
 *
 * Description:
 *   Undefined instruction entry exception.  Entered in UND mode, spsr = SVC  CPSR,
 *   lr = SVC PC
 *
 ****************************************************************************/

	.globl	arm_undefinedinsn
	.globl	arm_vectorundefinsn
	.type	arm_vectorundefinsn, %function

arm_vectorundefinsn:
	/* On entry we are free to use the UND mode registers
	 * r13 and r14
	 */

	mov	r13, #(PSR_MODE_SVC | PSR_I_BIT)
	msr	cpsr_c, r13		/* Switch to SVC mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SVC mode regs */

	mov	r0, #(PSR_MODE_UND | PSR_I_BIT)
	msr	cpsr_c, r0		/* Switch back UND mode */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	mov	r3, lr
	mrs	r4, spsr

	/* Then switch back to SVC mode */

	mov	r0, #(PSR_MODE_SVC | PSR_I_BIT)
	msr	cpsr_c, r0

	/* Get the correct values of SVC r13(sp) and r14(lr) in r1 and r2 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

	/* Then call the undef insn handler with interrupts disabled.
	 * void arm_undefinedinsn(struct xcptcontext *xcp)
	 */

	mov	fp, #0			/* Init frame pointer */
	mov	r0, sp			/* Get r0=xcp */
	bl	arm_undefinedinsn	/* Call the handler */

	/* Restore the CPSR, SVC mode registers and return */

	ldr	r0, [sp, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr	spsr_cxsf, r0		/* Set the return mode SPSR */
	ldmia	sp, {r0-r15}^		/* Return */
	.size	arm_vectorundefinsn, . - arm_vectorundefinsn

	.align	5

/****************************************************************************
 * Name: arm_vectorfiq
 *
 * Description:
 *   Shouldn't happen
 *
 ****************************************************************************/

	.globl	arm_vectorfiq
	.type	arm_vectorfiq, %function

arm_vectorfiq:
	subs	pc, lr, #4
	.size	arm_vectorfiq, . - arm_vectorfiq

/****************************************************************************
 *  Name: g_intstackalloc/g_intstacktop
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.bss
	.balign	4

	.globl	g_intstackalloc
	.type	g_intstackalloc, object
	.globl	g_intstacktop
	.type	g_intstacktop, object

g_intstackalloc:
	.skip	(CONFIG_ARCH_INTERRUPTSTACK & ~3)
g_intstacktop:
	.size	g_intstacktop, 0
	.size	g_intstackalloc, (CONFIG_ARCH_INTERRUPTSTACK & ~3)

#endif
	.end

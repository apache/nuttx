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
 * Name: savefpu
 *
 * Description:
 *   Save the state of the floating point registers.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
	.macro	savefpu, out, tmp
	/* Store all floating point registers.  Registers are stored in numeric order,
	 * s0, s1, ... in increasing address order.
	 */

#ifdef CONFIG_ARM_DPFPU32
	vstmia.64	\out!, {d0-d15}		/* Save the full FP context */
	vstmia.64	\out!, {d16-d31}
#else
	vstmia		\out!, {s0-s31}		/* Save the full FP context */
#endif

	/* Store the floating point control and status register.  At the end of the
	 * vstmia, r1 will point to the FPSCR storage location.
	 */

	vmrs		\tmp, fpscr		/* Fetch the FPSCR */
	str		\tmp, [\out], #4	/* Save the floating point control and status register */
	.endm
#endif

/****************************************************************************
 * Name: restorefpu
 *
 * Description:
 *   Restore the state of the floating point registers.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_FPU
	.macro	restorefpu, in, tmp
	/* Load all floating point registers.  Registers are loaded in numeric order,
	 * s0, s1, ... in increasing address order.
	 */

#ifdef CONFIG_ARM_DPFPU32
	vldmia.64	\in!, {d0-d15}		/* Restore the full FP context */
	vldmia.64	\in!, {d16-d31}
#else
	vldmia		\in!, {s0-s31}		/* Restore the full FP context */
#endif

	/* Load the floating point control and status register.   At the end of the
	 * vstmia, \in will point to the FPSCR storage location.
	 */

	ldr		\tmp, [\in], #4		/* Fetch the floating point control and status register */
	vmsr		fpscr, \tmp		/* Restore the FPSCR */
	.endm
#endif

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

	mov	r13, #(PSR_MODE_SYS | PSR_I_BIT)
	msr	cpsr_c, r13		/* Switch to SYS mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SYS mode regs */

	mov	r0, #(PSR_MODE_IRQ | PSR_I_BIT)
	msr	cpsr_c, r0			/* Switch back IRQ mode */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	sub	r3, lr, #4
	mrs	r4, spsr

	/* Then switch back to SYS mode */

	orr	r0, r0, #(PSR_MODE_SYS | PSR_I_BIT)
	msr	cpsr_c, r0

	/* Get the correct values of USR/SYS r13(sp) and r14(lr) in r1 and r2 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

#ifdef CONFIG_ARCH_FPU
	/* Save the state of the floating point registers. */

	add	r0, sp, #(4*REG_S0)	/* R1=Address of FP register storage */
	savefpu	r0, r1
#endif

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

	mov		r4, sp		/* Save the SP in a preserved register */

	/* If the interrupt stack is disabled, reserve xcpcontext to ensure
	 * that signal processing can have a separate xcpcontext to handle
	 * signal context (reference: arm_schedulesigaction.c):
	 *      ----------------------
	 *     |    IRQ XCP context   |
	 *      -------------------
	 *     |  Signal XCP context  |
	 *      ----------------------   <- SP
	 * also the sp should be restore from r4 after arm_doirq()
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE	/* Reserve signal context */

	bic	sp, sp, #7		/* Force 8-byte alignment */
	bl	arm_decodeirq		/* Call the handler */
	mov	sp, r4			/* Restore the possibly unaligned stack pointer */
#endif

#ifdef CONFIG_ARCH_FPU
	/* Restore the state of the floating point registers. */

	add	r1, r0, #(4*REG_S0)	/* R1=Address of FP register storage */
	restorefpu	r1, r2
#endif

	/* Switch back IRQ mode and return with shadow SPSR */

	mov	r4, #(PSR_MODE_IRQ | PSR_I_BIT)
	msr	cpsr_c, r4		/* Switch back IRQ mode */

	/* Restore the CPSR, SYS mode registers and return */

	ldr	r0, [sp, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr	spsr_cxsf, r0		/* Set the return mode SPSR */

	/* Life is simple when everything is IRQ mode */

	mov	r14, sp			/* (IRQ) r14=Register storage area */
	ldmia	r14!, {r0-r12}		/* Restore common r0-r12 */
	ldmia	r14, {r13, r14}^	/* Restore user mode r13/r14 */
	add	r14, r14, #(4*2)	/* (IRQ) r14=address of r15 storage */
	ldmia	r14, {r15}^		/* Return */

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
 *   SVC interrupt. We enter the SVC in SYS mode.
 *
 ****************************************************************************/

	.globl	arm_syscall
	.globl	arm_vectorsvc
	.type	arm_vectorsvc, %function

arm_vectorsvc:
	/* On entry, we are in SVC mode.  We are free to use the SVC mode r13
	 * and r14.
	 */

	mov	r13, #(PSR_MODE_SYS | PSR_I_BIT)
	msr	cpsr_c, r13		/* Switch to SYS mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SYS mode regs */

	mov	r0, #(PSR_MODE_SVC | PSR_I_BIT)
	msr	cpsr_c, r0			/* Switch back SVC mode */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	mov	r3, r14			/* Save r14 as the PC as well */
	mrs	r4, spsr		/* Get the saved CPSR */

	/* Then switch back to SYS mode */

	orr	r0, r0, #(PSR_MODE_SYS | PSR_I_BIT)
	msr	cpsr_c, r0

	/* Get the correct values of USR/SYS r13(sp) and r14(lr) in r1 and r2 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

#ifdef CONFIG_ARCH_FPU
	/* Save the state of the floating point registers. */

	add	r0, sp, #(4*REG_S0)	/* R1=Address of FP register storage */
	savefpu	r0, r1
#endif

	/* Then call the SVC handler with interrupts disabled.
	 * void arm_syscall(struct xcptcontext *xcp)
	 */

	mov	fp, #0			/* Init frame pointer */
	mov	r0, sp			/* Get r0=xcp */
	bl	arm_syscall		/* Call the handler */

#ifdef CONFIG_ARCH_FPU
	/* Restore the state of the floating point registers. */

	add	r1, r0, #(4*REG_S0)	/* R1=Address of FP register storage */
	restorefpu	r1, r2
#endif

	/* Switch back SVC mode and return with shadow SPSR */

	mov	r4, #(PSR_MODE_SVC | PSR_I_BIT)
	msr	cpsr_c, r4		/* Switch back SVC mode */

	/* Restore the CPSR, SYS mode registers and return */

	ldr	r0, [sp, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr	spsr_cxsf, r0		/* Set the return mode SPSR */

	/* Life is simple when everything is SVC mode */

	mov	r14, sp			/* (SVC) r14=Register storage area */
	ldmia	r14!, {r0-r12}		/* Restore common r0-r12 */
	ldmia	r14, {r13, r14}^	/* Restore user mode r13/r14 */
	add	r14, r14, #(4*2)	/* (SVC) r14=address of r15 storage */
	ldmia	r14, {r15}^		/* Return */
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

	mov	r13, #(PSR_MODE_SYS | PSR_I_BIT)
	msr	cpsr_c, r13		/* Switch to SYS mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SYS mode regs */

	mov	r0, #(PSR_MODE_ABT | PSR_I_BIT)
	msr	cpsr_c, r0		/* Switch back ABT mode */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	sub	r3, lr, #8
	mrs	r4, spsr

	/* Then switch back to SYS mode */

	mov	r0, #(PSR_MODE_SYS | PSR_I_BIT)
	msr	cpsr_c, r0

	/* Get the correct values of USR/SYS r13(sp) and r14(lr) in r1 and r2 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

#ifdef CONFIG_ARCH_FPU
	/* Save the state of the floating point registers. */

	add	r0, sp, #(4*REG_S0)	/* R1=Address of FP register storage */
	savefpu	r0, r1
#endif

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

#ifdef CONFIG_ARCH_FPU
	/* Restore the state of the floating point registers. */

	add	r1, r0, #(4*REG_S0)	/* R1=Address of FP register storage */
	restorefpu	r1, r2
#endif

	/* Switch back ABT mode and return with shadow SPSR */

	mov	r4, #(PSR_MODE_ABT | PSR_I_BIT)
	msr	cpsr_c, r4		/* Switch back ABT mode */

	/* Restore the CPSR, SYS mode registers and return */

	ldr	r0, [sp, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr	spsr_cxsf, r0		/* Set the return mode SPSR */

	/* Life is simple when everything is ABT mode */

	mov	r14, sp			/* (ABT) r14=Register storage area */
	ldmia	r14!, {r0-r12}		/* Restore common r0-r12 */
	ldmia	r14, {r13, r14}^	/* Restore user mode r13/r14 */
	add	r14, r14, #(4*2)	/* (ABT) r14=address of r15 storage */
	ldmia	r14, {r15}^		/* Return */
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

	mov	r13, #(PSR_MODE_SYS | PSR_I_BIT)
	msr	cpsr_c, r13		/* Switch to SYS mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SYS mode regs */

	mov	r0, #(PSR_MODE_ABT | PSR_I_BIT)
	msr	cpsr_c, r0		/* Switch back ABT mode */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	sub	r3, lr, #4
	mrs	r4, spsr

	/* Then switch back to SYS mode */

	mov	r0, #(PSR_MODE_SYS | PSR_I_BIT)
	msr	cpsr_c, r0

	/* Get the correct values of USR/SYS r13(sp) and r14(lr) in r1 and r2 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

#ifdef CONFIG_ARCH_FPU
	/* Save the state of the floating point registers. */

	add	r0, sp, #(4*REG_S0)	/* R1=Address of FP register storage */
	savefpu	r0, r1
#endif

	/* Then call the prefetch abort handler with interrupts disabled.
	 * void arm_prefetchabort(struct xcptcontext *xcp)
	 */

	mov	fp, #0			/* Init frame pointer */
	mov	r0, sp			/* Get r0=xcp */
	bl	arm_prefetchabort	/* Call the handler */

#ifdef CONFIG_ARCH_FPU
	/* Restore the state of the floating point registers. */

	add	r1, r0, #(4*REG_S0)	/* R1=Address of FP register storage */
	restorefpu	r1, r2
#endif

	/* Switch back ABT mode and return with shadow SPSR */

	mov	r4, #(PSR_MODE_ABT | PSR_I_BIT)
	msr	cpsr_c, r4		/* Switch back ABT mode */

	/* Restore the CPSR, SYS mode registers and return */

	ldr	r0, [sp, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr	spsr_cxsf, r0		/* Set the return mode SPSR */

	/* Life is simple when everything is ABT mode */

	mov	r14, sp			/* (ABT) r14=Register storage area */
	ldmia	r14!, {r0-r12}		/* Restore common r0-r12 */
	ldmia	r14, {r13, r14}^	/* Restore user mode r13/r14 */
	add	r14, r14, #(4*2)	/* (ABT) r14=address of r15 storage */
	ldmia	r14, {r15}^		/* Return */
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

	mov	r13, #(PSR_MODE_SYS | PSR_I_BIT)
	msr	cpsr_c, r13		/* Switch to SYS mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub	sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}		/* Save the SYS mode regs */

	mov	r0, #(PSR_MODE_UND | PSR_I_BIT)
	msr	cpsr_c, r0		/* Switch back UND mode */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	mov	r3, lr
	mrs	r4, spsr

	/* Then switch back to SYS mode */

	mov	r0, #(PSR_MODE_SYS | PSR_I_BIT)
	msr	cpsr_c, r0

	/* Get the correct values of USR/SYS r13(sp) and r14(lr) in r1 and r2 */

	add	r1, sp, #XCPTCONTEXT_SIZE
	mov	r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add	r0, sp, #(4*REG_SP)	/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

#ifdef CONFIG_ARCH_FPU
	/* Save the state of the floating point registers. */

	add	r0, sp, #(4*REG_S0)	/* R1=Address of FP register storage */
	savefpu	r0, r1
#endif

	/* Then call the undef insn handler with interrupts disabled.
	 * void arm_undefinedinsn(struct xcptcontext *xcp)
	 */

	mov	fp, #0			/* Init frame pointer */
	mov	r0, sp			/* Get r0=xcp */
	bl	arm_undefinedinsn	/* Call the handler */

#ifdef CONFIG_ARCH_FPU
	/* Restore the state of the floating point registers. */

	add	r1, r0, #(4*REG_S0)	/* R1=Address of FP register storage */
	restorefpu	r1, r2
#endif

	/* Switch back UND mode and return with shadow SPSR */

	mov	r4, #(PSR_MODE_UND | PSR_I_BIT)
	msr	cpsr_c, r4		/* Switch back UND mode */

	/* Restore the CPSR, SYS mode registers and return */

	ldr	r0, [sp, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr	spsr_cxsf, r0		/* Set the return mode SPSR */

	/* Life is simple when everything is UND mode */

	mov	r14, sp			/* (FIQ) r14=Register storage area */
	ldmia	r14!, {r0-r7}		/* Restore common r0-r7 */
	ldmia	r14, {r8-r14}^		/* Restore user mode r8-r14 */
	add	r14, r14, #(4*7)	/* (FIQ) r14=address of r15 storage */
	ldmia	r14, {r15}^		/* Return */
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

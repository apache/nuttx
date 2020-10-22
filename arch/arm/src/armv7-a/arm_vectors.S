/************************************************************************************
 * arch/arm/src/armv7-a/arm_vectors.S
 *
 *   Copyright (C) 2013-2014, 2018 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/irq.h>

#include "arm.h"
#include "cp15.h"
#include "chip.h"

	.file	"arm_vectors.S"

/************************************************************************************
 * Private Data
 ************************************************************************************/

	.data
g_irqtmp:
	.word	0		/* Saved lr */
	.word	0		/* Saved spsr */

g_undeftmp:
	.word	0		/* Saved lr */
	.word	0		/* Saved spsr */

g_aborttmp:
	.word	0		/* Saved lr */
	.word	0		/* Saved spsr */

#ifdef CONFIG_ARMV7A_DECODEFIQ
g_fiqtmp:
	.word	0		/* Saved lr */
	.word	0		/* Saved spsr */
#endif

#if CONFIG_ARCH_INTERRUPTSTACK > 7 && defined(CONFIG_ARMV7A_HAVE_GICv2)

g_nestlevel:
#ifdef CONFIG_SMP
	.rept	CONFIG_SMP_NCPUS
	.word	0		/* CPUn Interrupt nesting level */
	.endr
#else
	.word	0		/* Interrupt nesting level */
#endif

#endif /* CONFIG_ARCH_INTERRUPTSTACK > 7 && CONFIG_ARMV7A_HAVE_GICv2 */

/************************************************************************************
 * Macro Definitions
 ************************************************************************************/

/************************************************************************************
 * Name: cpuindex
 *
 * Description:
 *   Return an index idenifying the current CPU.  Single CPU case.  Must be
 *   provided by MCU-specific logic in chip.h for the SMP case.
 *
 ************************************************************************************/

#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.macro	cpuindex, index
	.mov	\index, #0
	.endm
#endif

/************************************************************************************
 * Name: setirqstack
 *
 * Description:
 *   Set the current stack pointer to the "top" of the IRQ interrupt stack.  Single
 *   CPU case.  Must be provided by MCU-specific logic in chip.h for the SMP case.
 *
 ************************************************************************************/

#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.macro	setirqstack, tmp1, tmp2
	ldr		sp, .Lirqstackbase		/* SP = IRQ stack top */
	.endm
#endif

/************************************************************************************
 * Name: setfiqstack
 *
 * Description:
 *   Set the current stack pointer to the "top" of the FIQ interrupt stack.  Single
 *   CPU case.  Must be provided by MCU-specific logic in chip.h for the SMP case.
 *
 ************************************************************************************/

#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.macro	setfiqstack, tmp1, tmp2
	ldr		sp, .Lfiqstackbase		/* SP = FIQ stack top */
	.endm
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

	.text

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: arm_vectorirq
 *
 * Description:
 *   Interrupt exception. Entered in IRQ mode with spsr = SVC CPSR, lr = SVC PC
 *
 ************************************************************************************/

	.globl	arm_decodeirq
	.globl	arm_vectorirq
	.type	arm_vectorirq, %function

arm_vectorirq:
	/* On entry, we are in IRQ mode.  We are free to use the IRQ mode r13
	 * and r14.
	 */

	ldr		r13, .Lirqtmp
	sub		lr, lr, #4
	str		lr, [r13]				/* Save lr_IRQ */
	mrs		lr, spsr
	str		lr, [r13, #4]			/* Save spsr_IRQ */

	/* Then switch back to SVC mode */

	bic		lr, lr, #PSR_MODE_MASK	/* Keep F and T bits */
#ifdef CONFIG_ARMV7A_DECODEFIQ
	orr		lr, lr, #(PSR_MODE_SVC | PSR_I_BIT | PSR_F_BIT)
#else
	orr		lr, lr, #(PSR_MODE_SVC | PSR_I_BIT)
#endif
	msr		cpsr_c, lr				/* Switch to SVC mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub		sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}			/* Save the SVC mode regs */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	ldr		r0, .Lirqtmp			/* Points to temp storage */
	ldmia	r0, {r3, r4}			/* Recover r3=lr_IRQ, r4=spsr_IRQ */

#ifdef CONFIG_BUILD_KERNEL
	/* Did we enter from user mode?  If so then we need get the values of
	 * USER mode r13(sp) and r14(lr).
	 */

	and		r1, r4, #PSR_MODE_MASK	/* Interrupted mode */
	cmp		r1, #PSR_MODE_USR		/* User mode? */
	bne		.Lirqentersvc			/* Branch if not user mode */

	/* ldmia with ^ will return the user mode registers (provided that r15
	 * is not in the register list).
	 */

	add		r0, sp, #(4*REG_SP)		/* Offset to sp/lr storage */
	stmia	r0, {r13, r14}^			/* Save user mode r13(sp) and r14(lr) */
	add		r0, sp, #(4*REG_R15)	/* Offset to pc/cpsr storage */
	stmia	r0, {r3, r4}			/* Save r15(pc), and the CPSR */
	b		.Lirqcontinue

.Lirqentersvc:
	/* Otherwise, get the correct values of SVC r13(sp) and r14(lr) in r1
	 * and r2.
	 */

	add		r1, sp, #XCPTCONTEXT_SIZE
	mov		r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add		r0, sp, #(4*REG_SP)		/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

.Lirqcontinue:

#else
	/* Get the correct values of SVC r13(sp) and r14(lr) in r1 and r2 */

	add		r1, sp, #XCPTCONTEXT_SIZE
	mov		r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add		r0, sp, #(4*REG_SP)		/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}
#endif

	/* Then call the IRQ handler with interrupts disabled. */

	mov		fp, #0					/* Init frame pointer */
	mov		r0, sp					/* Get r0=xcp */

#if CONFIG_ARCH_INTERRUPTSTACK > 7
#ifdef CONFIG_ARMV7A_HAVE_GICv2
	/* We will switch to the interrupt stack, UNLESS we are processing a
	 * nested interrupt in which case we are already using the interrupt
	 * stack.  SGI interrupts may be nested because they are non-maskable.
	 */

	ldr		r5, .Lirqnestlevel		/* r5=Points to interrupt nesting level */
#ifdef CONFIG_SMP
	cpuindex r1						/* r1=cpu index */
	lsls	r1, r1, #2				/* r1=array byte offset */
	add		r5, r5, r1				/* Indexed interrupt nesting level */
#endif
	ldr		r1, [r5]				/* Get r1= nesting level */
	add		r1, r1, #1				/* Increment nesting level */
	str		r1, [r5]				/* Save r1= nesting level */

	cmp		r1, #1					/* r1>1 if nested */
	bgt		.Lintnested				/* Use current SP if nested */
#endif

	/* Call arm_decodeirq() on the interrupt stack */

	setirqstack r1, r3				/* SP = IRQ stack top */
	str		r0, [sp, #-4]!			/* Save the xcp address at SP-4 then update SP */
	mov		r4, sp					/* Save the SP in a preserved register */
	bic		sp, sp, #7				/* Force 8-byte alignment */
	bl		arm_decodeirq			/* Call the handler */
	ldr		sp, [r4]				/* Restore the user stack pointer */

#ifdef CONFIG_ARMV7A_HAVE_GICv2
	b		.Lintreturn

	/* Call arm_decodeirq() on whatever stack is in place */

.Lintnested:
	mov		r4, sp					/* Save the SP in a preserved register */
	bic		sp, sp, #7				/* Force 8-byte alignment */
	bl		arm_decodeirq			/* Call the handler */
	mov		sp, r4					/* Restore the possibly unaligned stack pointer */

	/* Decrement the nesting level (r5 should be preserved) */

.Lintreturn:
	ldr		r1, [r5]				/* Get r1= nesting level */
	cmp		r1, #0					/* A sanity check*/
	subgt	r1, r1, #1				/* Decrement nesting level */
	strgt	r1, [r5]				/* Save r1= nesting level */
#endif

#else
	/* Call arm_decodeirq() on the user stack */

	mov		r4, sp					/* Save the SP in a preserved register */
	bic		sp, sp, #7				/* Force 8-byte alignment */
	bl		arm_decodeirq			/* Call the handler */
	mov		sp, r4					/* Restore the possibly unaligned stack pointer */
#endif

	/* Upon return from arm_decodeirq, r0 holds the pointer to the register
	 * state save area to use to restore the registers.  This may or may not
	 * be the same value that was passed to arm_decodeirq:  It will differ if a
	 * context switch is required.
	 */

	/* Restore the CPSR, SVC mode registers and return */

	ldr		r1, [r0, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr		spsr, r1				/* Set the return mode SPSR */

#ifdef CONFIG_BUILD_KERNEL
	/* Are we leaving in user mode?  If so then we need to restore the
	 * values of USER mode r13(sp) and r14(lr).
	 */

	and		r2, r1, #PSR_MODE_MASK	/* Interrupted mode */
	cmp		r2, #PSR_MODE_USR		/* User mode? */
	bne		.Lirqleavesvc			/* Branch if not user mode */

	/* ldmia with ^ will return the user mode registers (provided that r15
	 * is not in the register list).
	 */

	mov		r13, r0					/* (SVC) R13=Register storage area */
	ldmia	r13, {r0-R12}			/* Restore common R0-R12 */
	add		r14, r13, #(4*REG_R13)	/* (SVC) R14=address of R13/R14 storage */
	ldmia	r14, {r13, r14}^		/* Restore user mode R13/R14 */
	add		r14, r13, #(4*REG_R15)	/* (SVC) R14=address of R15 storage */
	ldmia	r14, {r15}^				/* Return */

.Lirqleavesvc:
#endif
	/* Life is simple when everything is SVC mode */

	ldmia	r0, {r0-r15}^			/* Return */

.Lirqtmp:
	.word	g_irqtmp

#if CONFIG_ARCH_INTERRUPTSTACK > 7
#ifndef CONFIG_SMP
.Lirqstackbase:
	.word	g_intstackbase
#endif

#ifdef CONFIG_ARMV7A_HAVE_GICv2
.Lirqnestlevel:
	.word	g_nestlevel
#endif
#endif /* CONFIG_ARCH_INTERRUPTSTACK > 7 */

	.size	arm_vectorirq, . - arm_vectorirq
	.align	5

/************************************************************************************
 * Function: arm_vectorsvc
 *
 * Description:
 *   SVC interrupt. We enter the SVC in SVC mode.
 *
 ************************************************************************************/

	.globl	arm_syscall
	.globl	arm_vectorsvc
	.type	arm_vectorsvc, %function

arm_vectorsvc:

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub		sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}			/* Save the SVC mode regs */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	mov		r3, r14					/* Save r14 as the PC as well */
	mrs		r4, spsr				/* Get the saved CPSR */

#ifdef CONFIG_BUILD_KERNEL
	/* Did we enter from user mode?  If so then we need get the values of
	 * USER mode r13(sp) and r14(lr).
	 */

	and		r1, r4, #PSR_MODE_MASK	/* Interrupted mode */
	cmp		r1, #PSR_MODE_USR		/* User mode? */
	bne		.Lsvcentersvc			/* Branch if not user mode */

	/* ldmia with ^ will return the user mode registers (provided that r15
	 * is not in the register list).
	 */

	add		r0, sp, #(4*REG_SP)		/* Offset to sp/lr storage */
	stmia	r0, {r13, r14}^			/* Save user mode r13(sp) and r14(lr) */
	add		r0, sp, #(4*REG_R15)	/* Offset to pc/cpsr storage */
	stmia	r0, {r3, r4}			/* Save r15(pc), and the CPSR */
	b		.Lsvccontinue

.Lsvcentersvc:
	/* Otherwise, get the correct values of SVC r13(sp) and r14(lr) in r1
	 * and r2.
	 */

	add		r1, sp, #XCPTCONTEXT_SIZE
	mov		r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add		r0, sp, #(4*REG_SP)		/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

.Lsvccontinue:

#else
	/* Get the correct values of SVC r13(sp) and r14(lr) in r1 and r2 */

	add		r1, sp, #XCPTCONTEXT_SIZE
	mov		r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add		r0, sp, #(4*REG_SP)		/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}
#endif

	/* Then call the SVC handler with interrupts disabled.
	 * void arm_syscall(struct xcptcontext *xcp)
	 */

	mov		fp, #0					/* Init frame pointer */
	mov		r0, sp					/* Get r0=xcp */
	mov		r4, sp					/* Save the SP in a preserved register */
	bic		sp, sp, #7				/* Force 8-byte alignment */
	bl		arm_syscall				/* Call the handler */
	mov		sp, r4					/* Restore the possibly unaligned stack pointer */

	/* Upon return from arm_syscall, r0 holds the pointer to the register
	 * state save area to use to restore the registers.  This may or may not
	 * be the same value that was passed to arm_syscall:  It will differ if a
	 * context switch is required.
	 */

	/* Restore the CPSR, SVC mode registers and return */

	ldr		r1, [r0, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr		spsr, r1				/* Set the return mode SPSR */

#ifdef CONFIG_BUILD_KERNEL
	/* Are we leaving in user mode?  If so then we need to restore the
	 * values of USER mode r13(sp) and r14(lr).
	 */

	and		r2, r1, #PSR_MODE_MASK	/* Interrupted mode */
	cmp		r2, #PSR_MODE_USR		/* User mode? */
	bne		.Lleavesvcsvc			/* Branch if not user mode */

	/* ldmia with ^ will return the user mode registers (provided that r15
	 * is not in the register list).
	 */

	mov		r13, r0					/* (SVC) R13=Register storage area */
	ldmia	r13, {r0-R12}			/* Restore common R0-R12 */
	add		r14, r13, #(4*REG_R13)	/* (SVC) R14=address of R13/R14 storage */
	ldmia	r14, {r13, r14}^		/* Restore user mode R13/R14 */
	add		r14, r13, #(4*REG_R15)	/* (SVC) R14=address of R15 storage */
	ldmia	r14, {r15}^				/* Return */

.Lleavesvcsvc:
#endif
	/* Life is simple when everything is SVC mode */

	ldmia	r0, {r0-r15}^			/* Return */

	.size	arm_vectorsvc, . - arm_vectorsvc

	.align	5

/************************************************************************************
 * Name: arm_vectordata
 *
 * Description:
 *   This is the data abort exception dispatcher. The ARM data abort exception occurs
 *   when a memory fault is detected during a data transfer.  This handler saves the
 *   current processor state and gives control to data abort handler.  This function
 *   is entered in ABORT mode with spsr = SVC CPSR, lr = SVC PC
 *
 ************************************************************************************/

	.globl	arm_dataabort
	.globl	arm_vectordata
	.type	arm_vectordata, %function

arm_vectordata:
	/* On entry we are free to use the ABORT mode registers
	 * r13 and r14
	 */

	ldr		r13, .Ldaborttmp		/* Points to temp storage */
	sub		lr, lr, #8				/* Fixup return */
	str		lr, [r13]				/* Save in temp storage */
	mrs		lr, spsr				/* Get SPSR */
	str		lr, [r13, #4]			/* Save in temp storage */

	/* Then switch back to SVC mode */

	bic		lr, lr, #PSR_MODE_MASK	/* Keep F and T bits */
	orr		lr, lr, #(PSR_MODE_SVC | PSR_I_BIT | PSR_F_BIT)
	msr		cpsr_c, lr				/* Switch to SVC mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub		sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}			/* Save the SVC mode regs */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	ldr		r0, .Ldaborttmp			/* Points to temp storage */
	ldmia	r0, {r3, r4}			/* Recover r3=lr_ABT, r4=spsr_ABT */

#ifdef CONFIG_BUILD_KERNEL
	/* Did we enter from user mode?  If so then we need get the values of
	 * USER mode r13(sp) and r14(lr).
	 */

	and		r1, r4, #PSR_MODE_MASK	/* Interrupted mode */
	cmp		r1, #PSR_MODE_USR		/* User mode? */
	bne		.Ldabtentersvc			/* Branch if not user mode */

	/* ldmia with ^ will return the user mode registers (provided that r15
	 * is not in the register list).
	 */

	add		r0, sp, #(4*REG_SP)		/* Offset to sp/lr storage */
	stmia	r0, {r13, r14}^			/* Save user mode r13(sp) and r14(lr) */
	add		r0, sp, #(4*REG_R15)	/* Offset to pc/cpsr storage */
	stmia	r0, {r3, r4}			/* Save r15(pc), and the CPSR */
	b		.Ldabtcontinue

.Ldabtentersvc:
	/* Otherwise, get the correct values of SVC r13(sp) and r14(lr) in r1
	 * and r2.
	 */

	add		r1, sp, #XCPTCONTEXT_SIZE
	mov		r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add		r0, sp, #(4*REG_SP)		/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

.Ldabtcontinue:

#else
	/* Get the correct values of SVC r13(sp) and r14(lr) in r1 and r2 */

	add		r1, sp, #XCPTCONTEXT_SIZE
	mov		r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add		r0, sp, #(4*REG_SP)		/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}
#endif

	/* Then call the data abort handler with interrupts disabled.
	 * void arm_dataabort(struct xcptcontext *xcp)
	 */

	mov		fp, #0					/* Init frame pointer */
	mov		r0, sp					/* Get r0=xcp */
	mrc		CP15_DFAR(r1)			/* Get R1=DFAR */
	mrc		CP15_DFSR(r2)			/* Get r2=DFSR */
	mov		r4, sp					/* Save the SP in a preserved register */
	bic		sp, sp, #7				/* Force 8-byte alignment */
	bl		arm_dataabort			/* Call the handler */
	mov		sp, r4					/* Restore the possibly unaligned stack pointer */

	/* Upon return from arm_dataabort, r0 holds the pointer to the register
	 * state save area to use to restore the registers.  This may or may not
	 * be the same value that was passed to arm_dataabort:  It will differ if a
	 * context switch is required.
	 */

	/* Restore the CPSR, SVC mode registers and return */

	ldr		r1, [r0, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr		spsr_cxsf, r1			/* Set the return mode SPSR */

#ifdef CONFIG_BUILD_KERNEL
	/* Are we leaving in user mode?  If so then we need to restore the
	 * values of USER mode r13(sp) and r14(lr).
	 */

	and		r2, r1, #PSR_MODE_MASK	/* Interrupted mode */
	cmp		r2, #PSR_MODE_USR		/* User mode? */
	bne		.Ldabtleavesvc			/* Branch if not user mode */

	/* ldmia with ^ will return the user mode registers (provided that r15
	 * is not in the register list).
	 */

	mov		r13, r0					/* (SVC) R13=Register storage area */
	ldmia	r13, {r0-R12}			/* Restore common R0-R12 */
	add		r14, r13, #(4*REG_R13)	/* (SVC) R14=address of R13/R14 storage */
	ldmia	r14, {r13, r14}^		/* Restore user mode R13/R14 */
	add		r14, r13, #(4*REG_R15)	/* (SVC) R14=address of R15 storage */
	ldmia	r14, {r15}^				/* Return */

.Ldabtleavesvc:
#endif
	/* Life is simple when everything is SVC mode */

	ldmia	r0, {r1-r15}^			/* Return */

.Ldaborttmp:
	.word	g_aborttmp
	.size	arm_vectordata, . - arm_vectordata

	.align	5

/************************************************************************************
 * Name: arm_vectorprefetch
 *
 * Description:
 *   This is the prefetch abort exception dispatcher. The ARM prefetch abort exception
 *   occurs when a memory fault is detected during an an instruction fetch.  This
 *   handler saves the current processor state and gives control to prefetch abort
 *   handler.  This function is entered in ABT mode with spsr = SVC CPSR, lr = SVC PC.
 *
 ************************************************************************************/

	.globl	arm_prefetchabort
	.globl	arm_vectorprefetch
	.type	arm_vectorprefetch, %function

arm_vectorprefetch:
	/* On entry we are free to use the ABORT mode registers
	 * r13 and r14
	 */

	ldr		r13, .Lpaborttmp		/* Points to temp storage */
	sub		lr, lr, #4				/* Fixup return */
	str		lr, [r13]				/* Save in temp storage */
	mrs		lr, spsr				/* Get SPSR */
	str		lr, [r13, #4]			/* Save in temp storage */

	/* Then switch back to SVC mode */

	bic		lr, lr, #PSR_MODE_MASK	/* Keep F and T bits */
	orr		lr, lr, #(PSR_MODE_SVC | PSR_I_BIT | PSR_F_BIT)
	msr		cpsr_c, lr				/* Switch to SVC mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub		sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}			/* Save the SVC mode regs */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	ldr		r0, .Lpaborttmp			/* Points to temp storage */
	ldmia	r0, {r3, r4}			/* Recover r3=lr_ABT, r4=spsr_ABT */

#ifdef CONFIG_BUILD_KERNEL
	/* Did we enter from user mode?  If so then we need get the values of
	 * USER mode r13(sp) and r14(lr).
	 */

	and		r1, r4, #PSR_MODE_MASK	/* Interrupted mode */
	cmp		r1, #PSR_MODE_USR		/* User mode? */
	bne		.Lpabtentersvc			/* Branch if not user mode */

	/* ldmia with ^ will return the user mode registers (provided that r15
	 * is not in the register list).
	 */

	add		r0, sp, #(4*REG_SP)		/* Offset to sp/lr storage */
	stmia	r0, {r13, r14}^			/* Save user mode r13(sp) and r14(lr) */
	add		r0, sp, #(4*REG_R15)	/* Offset to pc/cpsr storage */
	stmia	r0, {r3, r4}			/* Save r15(pc), and the CPSR */
	b		.Lpabtcontinue

.Lpabtentersvc:
	/* Otherwise, get the correct values of SVC r13(sp) and r14(lr) in r1
	 * and r2.
	 */

	add		r1, sp, #XCPTCONTEXT_SIZE
	mov		r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add		r0, sp, #(4*REG_SP)		/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

.Lpabtcontinue:

#else
	/* Get the correct values of SVC r13(sp) and r14(lr) in r1 and r2 */

	add		r1, sp, #XCPTCONTEXT_SIZE
	mov		r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add		r0, sp, #(4*REG_SP)		/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}
#endif

	/* Then call the prefetch abort handler with interrupts disabled.
	 * void arm_prefetchabort(struct xcptcontext *xcp)
	 */

	mov		fp, #0					/* Init frame pointer */
	mov		r0, sp					/* Get r0=xcp */
	mrc		CP15_IFAR(r1)			/* Get R1=IFAR */
	mrc		CP15_IFSR(r2)			/* Get r2=IFSR */
	mov		r4, sp					/* Save the SP in a preserved register */
	bic		sp, sp, #7				/* Force 8-byte alignment */
	bl		arm_prefetchabort		/* Call the handler */
	mov		sp, r4					/* Restore the possibly unaligned stack pointer */

	/* Upon return from arm_prefetchabort, r0 holds the pointer to the register
	 * state save area to use to restore the registers.  This may or may not
	 * be the same value that was passed to arm_prefetchabort:  It will differ if a
	 * context switch is required.
	 */

	/* Restore the CPSR, SVC mode registers and return */

	ldr		r1, [r0, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr		spsr_cxsf, r1			/* Set the return mode SPSR */

#ifdef CONFIG_BUILD_KERNEL
	/* Are we leaving in user mode?  If so then we need to restore the
	 * values of USER mode r13(sp) and r14(lr).
	 */

	and		r2, r1, #PSR_MODE_MASK	/* Interrupted mode */
	cmp		r2, #PSR_MODE_USR		/* User mode? */
	bne		.Lpabtleavesvc			/* Branch if not user mode */

	/* ldmia with ^ will return the user mode registers (provided that r15
	 * is not in the register list).
	 */

	mov		r13, r0					/* (SVC) R13=Register storage area */
	ldmia	r13, {r0-R12}			/* Restore common R0-R12 */
	add		r14, r13, #(4*REG_R13)	/* (SVC) R14=address of R13/R14 storage */
	ldmia	r14, {r13, r14}^		/* Restore user mode R13/R14 */
	add		r14, r13, #(4*REG_R15)	/* (SVC) R14=address of R15 storage */
	ldmia	r14, {r15}^				/* Return */

.Lpabtleavesvc:
#endif
	/* Life is simple when everything is SVC mode */

	ldmia	r0, {r0-r15}^			/* Return */

.Lpaborttmp:
	.word	g_aborttmp
	.size	arm_vectorprefetch, . - arm_vectorprefetch

	.align	5

/************************************************************************************
 * Name: arm_vectorundefinsn
 *
 * Description:
 *   Undefined instruction entry exception.  Entered in UND mode, spsr = SVC  CPSR,
 *   lr = SVC PC
 *
 ************************************************************************************/

	.globl	arm_undefinedinsn
	.globl	arm_vectorundefinsn
	.type	arm_vectorundefinsn, %function

arm_vectorundefinsn:
	/* On entry we are free to use the UND mode registers
	 * r13 and r14
	 */

	ldr		r13, .Lundeftmp			/* Points to temp storage */
	str		lr, [r13]				/* Save in temp storage */
	mrs		lr, spsr				/* Get SPSR */
	str		lr, [r13, #4]			/* Save in temp storage */

	/* Then switch back to SVC mode */

	bic		lr, lr, #PSR_MODE_MASK	/* Keep F and T bits */
	orr		lr, lr, #(PSR_MODE_SVC | PSR_I_BIT | PSR_F_BIT)
	msr		cpsr_c, lr				/* Switch to SVC mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub		sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}			/* Save the SVC mode regs */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	ldr		r0, .Lundeftmp			/* Points to temp storage */
	ldmia	r0, {r3, r4}			/* Recover r3=lr_UND, r4=spsr_UND */

#ifdef CONFIG_BUILD_KERNEL
	/* Did we enter from user mode?  If so then we need get the values of
	 * USER mode r13(sp) and r14(lr).
	 */

	and		r1, r4, #PSR_MODE_MASK	/* Interrupted mode */
	cmp		r1, #PSR_MODE_USR		/* User mode? */
	bne		.Lundefentersvc			/* Branch if not user mode */

	/* ldmia with ^ will return the user mode registers (provided that r15
	 * is not in the register list).
	 */

	add		r0, sp, #(4*REG_SP)		/* Offset to sp/lr storage */
	stmia	r0, {r13, r14}^			/* Save user mode r13(sp) and r14(lr) */
	add		r0, sp, #(4*REG_R15)	/* Offset to pc/cpsr storage */
	stmia	r0, {r3, r4}			/* Save r15(pc), and the CPSR */
	b		.Lundefcontinue

.Lundefentersvc:
	/* Otherwise, get the correct values of SVC r13(sp) and r14(lr) in r1
	 * and r2.
	 */

	add		r1, sp, #XCPTCONTEXT_SIZE
	mov		r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add		r0, sp, #(4*REG_SP)		/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

.Lundefcontinue:

#else
	/* Get the correct values of SVC r13(sp) and r14(lr) in r1 and r2 */

	add		r1, sp, #XCPTCONTEXT_SIZE
	mov		r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add		r0, sp, #(4*REG_SP)		/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}
#endif

	/* Then call the undef insn handler with interrupts disabled.
	 * void arm_undefinedinsn(struct xcptcontext *xcp)
	 */

	mov		fp, #0					/* Init frame pointer */
	mov		r0, sp					/* Get r0=xcp */
	mov		r4, sp					/* Save the SP in a preserved register */
	bic		sp, sp, #7				/* Force 8-byte alignment */
	bl		arm_undefinedinsn		/* Call the handler */
	mov		sp, r4					/* Restore the possibly unaligned stack pointer */

	/* Upon return from arm_undefinedinsn, r0 holds the pointer to the register
	 * state save area to use to restore the registers.  This may or may not
	 * be the same value that was passed to arm_undefinedinsn:  It will differ if a
	 * context switch is required.
	 */

	/* Restore the CPSR, SVC mode registers and return */

	ldr		r1, [r0, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr		spsr_cxsf, r1			/* Set the return mode SPSR */

#ifdef CONFIG_BUILD_KERNEL
	/* Are we leaving in user mode?  If so then we need to restore the
	 * values of USER mode r13(sp) and r14(lr).
	 */

	and		r2, r1, #PSR_MODE_MASK	/* Interrupted mode */
	cmp		r2, #PSR_MODE_USR		/* User mode? */
	bne		.Lundefleavesvc			/* Branch if not user mode */

	/* ldmia with ^ will return the user mode registers (provided that r15
	 * is not in the register list).
	 */

	mov		r13, r0					/* (SVC) R13=Register storage area */
	ldmia	r13, {r0-R12}			/* Restore common R0-R12 */
	add		r14, r13, #(4*REG_R13)	/* (SVC) R14=address of R13/R14 storage */
	ldmia	r14, {r13, r14}^		/* Restore user mode R13/R14 */
	add		r14, r13, #(4*REG_R15)	/* (SVC) R14=address of R15 storage */
	ldmia	r14, {r15}^				/* Return */

.Lundefleavesvc:
#endif
	/* Life is simple when everything is SVC mode */

	ldmia	r0, {r0-r15}^			/* Return */

.Lundeftmp:
	.word	g_undeftmp
	.size	arm_vectorundefinsn, . - arm_vectorundefinsn

	.align	5

/************************************************************************************
 * Name: arm_vectorfiq
 *
 * Description:
 *   Shouldn't happen unless a arm_decodefiq() is provided.  FIQ is primarily used
 *   with the TrustZone feature in order to handle secure interrupts.
 *
 ************************************************************************************/

#ifdef CONFIG_ARMV7A_DECODEFIQ
	.globl	arm_decodefiq
#endif
	.globl	arm_vectorfiq
	.type	arm_vectorfiq, %function

arm_vectorfiq:
#ifdef CONFIG_ARMV7A_DECODEFIQ
	/* On entry we are free to use the FIQ mode registers r8 through r14 */

	ldr		r13, .Lfiqtmp			/* Points to temp storage */
	sub		lr, lr, #4				/* Fixup return */
	str		lr, [r13]				/* Save in temp storage */
	mrs		lr, spsr				/* Get SPSR_fiq */
	str		lr, [r13, #4]			/* Save in temp storage */

	/* Then switch back to SVC mode */

	bic		lr, lr, #PSR_MODE_MASK	/* Keep F and T bits */
	orr		lr, lr, #(PSR_MODE_SVC | PSR_I_BIT | PSR_F_BIT)
	msr		cpsr_c, lr				/* Switch to SVC mode */

	/* Create a context structure.  First set aside a stack frame
	 * and store r0-r12 into the frame.
	 */

	sub		sp, sp, #XCPTCONTEXT_SIZE
	stmia	sp, {r0-r12}			/* Save the SVC mode regs */

	/* Get the values for r15(pc) and CPSR in r3 and r4 */

	ldr		r0, .Lfiqtmp			/* Points to temp storage */
	ldmia	r0, {r3, r4}			/* Recover r3=lr_SVC, r4=spsr_SVC */

#ifdef CONFIG_BUILD_KERNEL
	/* Did we enter from user mode?  If so then we need get the values of
	 * USER mode rr13(sp) and r14(lr).
	 */

	and		r1, r4, #PSR_MODE_MASK	/* Interrupted mode */
	cmp		r1, #PSR_MODE_USR		/* User mode? */
	bne		.Lfiqentersvc			/* Branch if not user mode */

	/* ldmia with ^ will return the user mode registers (provided that r15
	 * is not in the register list).
	 */

	add		r0, sp, #(4*REG_SP)		/* Offset to sp/lr storage */
	stmia	r0, {r13, r14}^			/* Save user mode r13(sp) and r14(lr) */
	add		r0, sp, #(4*REG_R15)	/* Offset to pc/cpsr storage */
	stmia	r0, {r3, r4}			/* Save r15(pc), and the CPSR */
	b		.Lfiqcontinue

.Lfiqentersvc:
	/* Otherwise, get the correct values of SVC r13(sp) and r14(lr) in r1
	 * and r2.
	 */

	add		r1, sp, #XCPTCONTEXT_SIZE
	mov		r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add		r0, sp, #(4*REG_SP)		/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}

.Lfiqcontinue:

#else
	/* Get the correct values of SVC r13(sp) and r14(lr) in r1 and r2 */

	add		r1, sp, #XCPTCONTEXT_SIZE
	mov		r2, r14

	/* Save r13(sp), r14(lr), r15(pc), and the CPSR */

	add		r0, sp, #(4*REG_SP)		/* Offset to pc, cpsr storage */
	stmia	r0, {r1-r4}
#endif

	/* Then call the FIQ handler with interrupts disabled. */

	mov		fp, #0					/* Init frame pointer */
	mov		r0, sp					/* Get r0=xcp */

#if CONFIG_ARCH_INTERRUPTSTACK > 7
	setfiqstack r1, r4				/* SP = FIQ stack top */
	str		r0, [sp, #-4]!			/* Save the xcp address at SP-4 then update SP */
	mov		r4, sp					/* Save the SP in a preserved register */
	bic		sp, sp, #7				/* Force 8-byte alignment */
	bl		arm_decodefiq			/* Call the handler */
	ldr		sp, [r4]				/* Restore the user stack pointer */
#else
	mov		r4, sp					/* Save the SP in a preserved register */
	bic		sp, sp, #7				/* Force 8-byte alignment */
	bl		arm_decodefiq			/* Call the handler */
	mov		sp, r4					/* Restore the possibly unaligned stack pointer */
#endif

	/* Upon return from arm_decodefiq, r0 holds the pointer to the register
	 * state save area to use to restore the registers.  This may or may not
	 * be the same value that was passed to arm_decodefiq:  It will differ if a
	 * context switch is required.
	 */

	/* Restore the CPSR, SVC mode registers and return */

	ldr		r1, [r0, #(4*REG_CPSR)]	/* Fetch the return SPSR */
	msr		spsr, r1				/* Set the return mode SPSR */

#ifdef CONFIG_BUILD_KERNEL
	/* Are we leaving in user mode?  If so then we need to restore the
	 * values of USER mode r13(sp) and r14(lr).
	 */

	and		r2, r1, #PSR_MODE_MASK	/* Interrupted mode */
	cmp		r2, #PSR_MODE_USR		/* User mode? */
	bne		.Lfiqleavesvc			/* Branch if not user mode */

	/* ldmia with ^ will return the user mode registers (provided that r15
	 * is not in the register list).
	 */

	mov		r13, r0					/* (SVC) R13=Register storage area */
	ldmia	r13, {r0-R12}			/* Restore common R0-R12 */
	add		r14, r13, #(4*REG_R13)	/* (SVC) R14=address of R13/R14 storage */
	ldmia	r14, {r13, r14}^		/* Restore user mode R13/R14 */
	add		r14, r13, #(4*REG_R15)	/* (SVC) R14=address of R15 storage */
	ldmia	r14, {r15}^				/* Return */

.Lfiqleavesvc:
#endif
	/* Life is simple when everything is SVC mode */

	ldmia	r0, {r0-r15}^			/* Return */

.Lfiqtmp:
	.word	g_fiqtmp

#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
.Lfiqstackbase:
	.word	g_fiqstackbase
#endif

#else
	subs	pc, lr, #4
#endif
	.size	arm_vectorfiq, . - arm_vectorfiq

/************************************************************************************
 *  Name: g_intstackalloc/g_intstackbase
 ************************************************************************************/

#if !defined(CONFIG_SMP) && CONFIG_ARCH_INTERRUPTSTACK > 7
	.bss
	.balign	8

	.globl	g_intstackalloc
	.type	g_intstackalloc, object
	.globl	g_intstackbase
	.type	g_intstackbase, object

g_intstackalloc:
	.skip	((CONFIG_ARCH_INTERRUPTSTACK + 4) & ~7)
g_intstackbase:
	.skip	4
	.size	g_intstackbase, 4
	.size	g_intstackalloc, (CONFIG_ARCH_INTERRUPTSTACK & ~7)

/************************************************************************************
 *  Name: g_fiqstackalloc/g_fiqstackbase
 ************************************************************************************/

	.globl	g_fiqstackalloc
	.type	g_fiqstackalloc, object
	.globl	g_fiqstackbase
	.type	g_fiqstackbase, object

g_fiqstackalloc:
	.skip	((CONFIG_ARCH_INTERRUPTSTACK + 4) & ~7)
g_fiqstackbase:
	.skip	4
	.size	g_fiqstackbase, 4
	.size	g_fiqstackalloc, (CONFIG_ARCH_INTERRUPTSTACK & ~7)

#endif /* !CONFIG_SMP && CONFIG_ARCH_INTERRUPTSTACK > 7 */
	.end

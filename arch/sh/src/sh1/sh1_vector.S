/*****************************************************************************
 * arch/sh/src/sh1/sh1_vector.S
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>	/* NuttX configuration settings */
#include <arch/board/board.h>	/* Board-specific settings */
#include <arch/irq.h>		/* IRQ definitons */

#include "chip.h"		/* Chip-specific settings */
#include "up_internal.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/

/*****************************************************************************
 * External references
 *****************************************************************************/

/* Called functions */

	.globl	_up_doirq		/* C interrupt processing logic */

/*****************************************************************************
 * Macros
 *****************************************************************************/

/************************************************************************************
 * Macro: trampoline
 *
 * Description:
 *   Enter on exception with:
 *
 *   SP -> PC
 *         SR
 *
 *   Branch to up_vector with:
 *
 *   R4  : IRQ vector number
 *   SP -> Saved R4
 *         Saved SP (R15)
 *         PC
 *         SR
 *
 ************************************************************************************/

	.macro	trampoline, irq, label
	mov.l	r4, @-r15		/* Save R4 on the stack */
	mov.w	.L\label, r4		/* With the IRQ number in R4 */
	bra	_up_vector		/* Jump to the common vector handling logic */
	nop
.L\label:
	.word	\irq
	.endm

/*****************************************************************************
 * Text
 *****************************************************************************/

	.section	.reset

/*****************************************************************************
 * Name: _up_*_handler
 *
 * Description:
 *   Trampoline entry points for each, individual IRQ
 *
 *  R4 :  Points to a the register save structure
 *
 *****************************************************************************/

	.globl	_up_invalid_handler
_up_invalid_handler:
	trampoline NR_IRQS, 1

#ifdef CONFIG_SH1_DMAC0
	.globl	_up_dmac0_handler
_up_dmac0_handler:
	trampoline SH1_DEI0_IRQ, 2 /* DEI0 */
#endif

#ifdef CONFIG_SH1_DMAC1
	.globl	_up_dmac1_handler
_up_dmac1_handler:
	trampoline SH1_DEI1_IRQ, 3 /* DEI1 */
#endif

#ifdef CONFIG_SH1_DMAC2
	.globl	_up_dmac2_handler
_up_dmac2_handler:
	trampoline SH1_DEI2_IRQ, 4 /* DEI2 */
#endif

#ifdef CONFIG_SH1_DMAC3
	.globl	_up_dmac3_handler
_up_dmac4_handler:
	trampoline SH1_DEI3_IRQ, 5 /* DEI3 */
#endif

	.globl	_up_imia0_handler
	.globl	_up_imib0_handler
	.globl	_up_ovi0_handler
_up_imia0_handler:
	trampoline SH1_IMIA0_IRQ, 6 /* IMIA0 */
_up_imib0_handler:
	trampoline SH1_IMIB0_IRQ, 7 /* IMIB0 */
_up_ovi0_handler:
	trampoline SH1_OVI0_IRQ, 8  /* OVI0 */

#ifdef CONFIG_SH1_ITU1
	.globl	_up_imia1_handler
	.globl	_up_imib1_handler
	.globl	_up_ovi1_handler
_up_imia1_handler:
	trampoline SH1_IMIA1_IRQ, 9 /* IMIA1 */
_up_imib1_handler:
	trampoline SH1_IMIB1_IRQ, 10 /* IMIB1 */
_up_ovi1_handler:
	trampoline SH1_OVI1_IRQ, 11  /* OVI1 */
#endif

#ifdef CONFIG_SH1_ITU2
	.globl	_up_imia2_handler
	.globl	_up_imib2_handler
	.globl	_up_ovi2_handler
_up_imia2_handler:
	trampoline SH1_IMIA2_IRQ, 12 /* IMIA2 */
_up_imib2_handler:
	trampoline SH1_IMIB2_IRQ, 13 /* IMIB2 */
_up_ovi2_handler:
	trampoline SH1_OVI2_IRQ, 14  /* OVI2 */
#endif

#ifdef CONFIG_SH1_ITU3
	.globl	_up_imia3_handler
	.globl	_up_imib3_handler
	.globl	_up_ovi3_handler
_up_imia3_handler:
	trampoline SH1_IMIA3_IRQ, 15 /* IMIA3 */
_up_imib3_handler:
	trampoline SH1_IMIB3_IRQ, 16 /* IMIB3 */
_up_ovi3_handler:
	trampoline SH1_OVI3_IRQ, 17  /* OVI3 */
#endif

#ifdef CONFIG_SH1_ITU4
	.globl	_up_imia4_handler
	.globl	_up_imib4_handler
	.globl	_up_ovi4_handler
_up_imia4_handler:
	trampoline SH1_IMIA4_IRQ, 18 /* IMIA4 */
_up_imib4_handler:
	trampoline SH1_IMIB4_IRQ, 19 /* IMIB4 */
_up_ovi4_handler:
	trampoline SH1_OVI4_IRQ, 20  /* OVI4 */
#endif

#ifdef CONFIG_SH1_SCI0
	.globl	_up_eri0_handler
	.globl	_up_rxi0_handler
	.globl	_up_txi0_handler
	.globl	_up_tei0_handler
_up_eri0_handler:
	trampoline SH1_ERI0_IRQ, 21 /*  ERI0 */
_up_rxi0_handler:
	trampoline SH1_RXI0_IRQ, 22 /*  RxI0 */
_up_txi0_handler:
	trampoline SH1_TXI0_IRQ, 23 /*  TxI0 */
_up_tei0_handler:
	trampoline SH1_TEI0_IRQ, 24 /*  TEI0 */
#endif

#ifdef CONFIG_SH1_SCI1
	.globl	_up_eri1_handler
	.globl	_up_rxi1_handler
	.globl	_up_txi1_handler
	.globl	_up_tei1_handler
_up_eri1_handler:
	trampoline SH1_ERI1_IRQ, 25 /*  ERI1 */
_up_rxi1_handler:
	trampoline SH1_RXI1_IRQ, 26 /*  RxI1 */
_up_txi1_handler:
	trampoline SH1_TXI1_IRQ, 27 /*  TxI1 */
_up_tei1_handler:
	trampoline SH1_TEI1_IRQ, 28 /*  TEI1 */
#endif

#ifdef CONFIG_SH1_PCU
	.globl	_up_pei_handler
_up_pei_handler:
	trampoline SH1_PEI_IRQ, 29 /* Parity control unit PEI */
#endif

#ifdef CONFIG_SH1_AD
	.globl	_up_aditi_handler
_up_aditi_handler:
	trampoline SH1_ADITI_IRQ, 30 /* A/D ITI */
#endif

#ifdef CONFIG_SH1_WDT
	.globl	_up_wdt_handler
_up_wdt_handler:
	trampoline SH1_WDTITI_IRQ, 31 /* WDT ITI */
#endif

#ifdef CONFIG_SH1_CMI
	.globl	_up_cmi_handler
_up_cmi_handler:
	trampoline SH1_CMI_IRQ,32 /* REF CMI */
#endif

/*****************************************************************************
 * Name: _up_fullcontextrestore
 *
 * Description:
 *   restore context from a set of save registers
 *
 *  R4 :  Points to a the register save structure
 *
 *****************************************************************************/

	.global _up_fullcontextrestore
	.type	_up_fullcontextrestore, #function

_up_fullcontextrestore:
	stc     sr, r8		/* Mask all interrupts */
	mov.l	.Lintmask, r9
	or	r9, r8
	ldc     r8, sr

	mov	r5, r15		/* Replace stack pointer with the context save */
	bra	_up_restoreregs
	.size	_up_fullcontextrestore, .-_up_fullcontextrestore

/*****************************************************************************
 * Name: _up_vector
 *
 * Description:
 *   Execption entry point.  Upon entry:
 *
 *  R4 :  Holds IRQ number
 *  SP -> Saved R4
 *        Saved SP (R15)
 *        PC
 *        SR
 *
 *****************************************************************************/

	.global _up_vector
	.type	_up_vector, #function

_up_vector:
	/* Save the registers on the stack. (R4 is already saved) */

	sts.l	macl, @-r15
	sts.l	mach, @-r15
	stc.l	gbr, @-r15
	sts.l	pr, @-r15

	mov.l	r14, @-r15
	mov.l	r13, @-r15
	mov.l	r12, @-r15
	mov.l	r11, @-r15
	mov.l	r10, @-r15
	mov.l	r9, @-r15
	mov.l	r8, @-r15

	stc     sr, r8			/* Mask all interrupts */
	mov.l	.Lintmask, r9
	or	r9, r8
	ldc     r8, sr

	mov.l	r7, @-r15
	mov.l	r6, @-r15
	mov.l	r5, @-r15
	mov.l	r3, @-r15
	mov.l	r2, @-r15
	mov.l	r1, @-r15
	mov.l	r0, @r15

	/* Setup parameters: R4=IRQ number, R5=base of saved state */

	mov	r15, r5

	/* Switch to the interrupt stack */

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	mov.l	.Lintstack, r15		/* SP = interrupt stack base */
	mov.l	r5, @sp			/* Save the user stack pointer */
#endif
	/* Dispatch the interrupt */

        mov.l   .Ldoirq, r0
        jsr     @r0
        mov     r15, r14

	/* On return, R0 holds the address of the base of the XCPTCONTEXT
	 * structure to use for the return (may not be the same as the
	 * one that we passed in via r5.
	 */

	add	#-XCPTCONTEXT_SIZE, r0
	mov	r0, r15

	/* Restore registers.  R15 may point to either the stack or a saved
	 * context structure inside of a TCB.
	 */

_up_restoreregs:
	mov.l	@r15+, r0
	mov.l	@r15+, r1
	mov.l	@r15+, r2
	mov.l	@r15+, r3
	mov.l	@r15+, r5
	mov.l	@r15+, r6
	mov.l	@r15+, r7
	mov.l	@r15+, r8
	mov.l	@r15+, r9
	mov.l	@r15+, r10
	mov.l	@r15+, r11
	mov.l	@r15+, r12
	mov.l	@r15+, r13
	mov.l	@r15+, r14

	lds.l	@r15+, pr
	ldc.l	@r15+, gbr
	lds.l	@r15+, mach
	lds.l	@r15+, macl

	mov.l	@r15+, r4
	mov.l	@r15, r15	/* The real stack pointer */
	rte
	nop

	.align	2
.Lintmask:
	.long	0x000000f0
#if CONFIG_ARCH_INTERRUPTSTACK > 3
.Lintstack:
	.long	_up_stackbase
#endif
.Ldoirq:
        .long   _up_doirq
        .size   _up_vector, .-_up_vector

/************************************************************************************
 *  Name: up_interruptstack/g_userstack
 *
 * Description:
 *   Shouldn't happen
 *
 ************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 3
	.bss
	.align	4
	.globl	_g_userstack
	.type	_g_userstack, object
_up_interruptstack:
	.skip	((CONFIG_ARCH_INTERRUPTSTACK & ~3) - 4)
_g_userstack:
_up_stackbase:
	.skip	4
	.size	_g_userstack, 4
	.size	_up_interruptstack, (CONFIG_ARCH_INTERRUPTSTACK & ~3)
#endif
	.end


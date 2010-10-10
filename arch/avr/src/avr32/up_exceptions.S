/****************************************************************************
 * arch/avr32/src/avr32/up_exceptions.S
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/avr32/avr32.h>

#include "up_internal.h"
#include "up_arch.h"

/****************************************************************************
 * External Symbols
 ****************************************************************************/

	.global		avr32_getirqno

/****************************************************************************
 * Macros
 ****************************************************************************/

#warning "Missing Logic"
	.macro  HANDLER, label, irqno
\label:
	rjmp	avr32_excptcommon	/* FIXME!!! Need IRQ in a register */
	.endm

/****************************************************************************
 * Exception Vector Table
 ****************************************************************************/

/* The Exception Vector Base Address (EVBA) register will contain "a pointer
 * to the exception routines. All exception routines start at this address,
 * or at a defined offset relative to the address. Special alignment
 * requirements may apply for EVBA, depending on the implementation of the
 * interrupt controller."
 */

/* REVISIT:  This alignment requirement may be different on other AVR32s */

	.text
	.balign 0x200

	.global	vectortab
	.type	vectortab, @function
vectortab:
	lda.w	pc, avr32_unrec			/* EVBA+0x00 Unrecoverable exception */
	lda.w	pc, avr32_tlbmult		/* EVBA+0x04 TLB multiple hit */
	lda.w	pc, avr32_busdata		/* EVBA+0x08 Bus error data fetch */
	lda.w	pc, avr32_businst		/* EVBA+0x0c Bus error instruction fetch */
	lda.w	pc, avr32_nmi			/* EVBA+0x10 NMI */
	lda.w	pc, avr32_instaddr		/* EVBA+0x14 Instruction Address */
	lda.w	pc, avr32_itlbrot		/* EVBA+0x18 ITLB Protection */
	lda.w	pc, avr32_bp			/* EVBA+0x1c Breakpoint */
	lda.w	pc, avr32_invinst		/* EVBA+0x20 Illegal Opcode */
	lda.w	pc, avr32_unimpinst		/* EVBA+0x24 Unimplemented instruction */
	lda.w	pc, avr32_priv			/* EVBA+0x28 Privilege violation */
	lda.w	pc, avr32_fp			/* EVBA+0x2c Floating-point */
	lda.w	pc, avr32_cop			/* EVBA+0x30 Coprocessor absent */
	lda.w	pc, avr32_rddata		/* EVBA+0x34 Data Address (Read) */
	lda.w	pc, avr32_wrdata		/* EVBA+0x38 Data Address (Write) */
	lda.w	pc, avr32_tddtlbprot	/* EVBA+0x3c DTLB Protection (Read) */
	lda.w	pc, avr32_wrdtlbprot	/* EVBA+0x40 DTLB Protection (Write) */
	lda.w	pc, avr32_dltbmod		/* EVBA+0x44 DTLB Modified */
	.rept	2
	lda.w	pc,	avr32_badvector		/* EVBA+0x48-0x4c No such vector */
	.endr
	lda.w	pc, avr32_itlbmiss		/* EVBA+0x50 ITLB Miss */
	.rept	3
	lda.w	pc,	avr32_badvector		/* EVBA+0x54-0x5c No such vector */
	.endr
	lda.w	pc, avr32_rddtlb		/* EVBA+0x60 DTLB Miss (Read) */
	.rept	3
	lda.w	pc,	avr32_badvector		/* EVBA+0x64-0x6c No such vector */
	.endr
	lda.w	pc, avr32_wrdtlb		/* EVBA+0x70 DTLB Miss (Write) */
	.rept	(3+4*8)
	lda.w	pc,	avr32_badvector		/* EVBA+0x74-0xfc No such vector */
	.endr
	lda.w	pc, avr32_super			/* EVBA+0x100 Supervisor call */

/****************************************************************************
 * Interrupts
 ****************************************************************************/

/* The interrupt controller must provide an address that is relative to the
 * the EVBA so it is natural to define these interrupt vectors just after
 * the exception table.
 * On entry to each interrupt handler, R8-R12, LR, PC and SR have already been
 * pushed onto the system stack by the MCU.
 */

	.balign 4
avr32_int0:
	mov		r12, 0				/* r12=interrupt level parameter */
	rjmp	avr32_intcommon		/* Jump to common interrupt logic */

	.balign	4
avr32_int1:
	mov		r12, 1				/* r12=interrupt level parameter */
	rjmp	avr32_intcommon		/* Jump to common interrupt logic */

	.balign	4
avr32_int2:
	mov		r12, 2				/* r12=interrupt level parameter */
	rjmp	avr32_intcommon		/* Jump to common interrupt logic */

avr32_int3:
	mov		r12, 2				/* r12=interrupt level parameter */

/* Common interrupt handling logic */

avr32_intcommon:
	call	avr32_getirqno		/* Get the IRQ number of the interrupt to process */
	rjmp	avr32_common		/* Then go to the common exception handling logic */

/****************************************************************************
 * Exception Vector Handlers
 ****************************************************************************/

 /* Exception Handlers */
 
	HANDLER	avr32_unrec, AVR32_IRQ_UNREC			/* Unrecoverable exception */
	HANDLER	avr32_tlbmult, AVR32_IRQ_TLBMULT		/* TLB multiple hit */
	HANDLER	avr32_busdata, AVR32_IRQ_BUSDATA		/* Bus error data fetch */
	HANDLER	avr32_businst, AVR32_IRQ_BUSINST		/* Bus error instruction fetch */
	HANDLER	avr32_nmi, AVR32_IRQ_NMI				/* NMI */
	HANDLER	avr32_instaddr, AVR32_IRQ_INSTADDR		/* Instruction Address */
	HANDLER	avr32_itlbrot, AVR32_IRQ_ITLBPROT		/* ITLB Protection */
	HANDLER	avr32_bp, AVR32_IRQ_BP					/* Breakpoint */
	HANDLER	avr32_invinst, AVR32_IRQ_INVINST		/* Illegal Opcode */
	HANDLER	avr32_unimpinst, AVR32_IRQ_UNIMPINST	/* Unimplemented instruction */
	HANDLER	avr32_priv, AVR32_IRQ_PRIV 				/* Privilege violation */
	HANDLER	avr32_fp, AVR32_IRQ_FP					/* Floating-point */
	HANDLER	avr32_cop, AVR32_IRQ_COP				/* Coprocessor absent */
	HANDLER	avr32_rddata, AVR32_IRQ_RDDATA			/* Data Address (Read) */
	HANDLER	avr32_wrdata, AVR32_IRQ_WRDATA 			/* Data Address (Write) */
	HANDLER	avr32_tddtlbprot, AVR32_IRQ_RDDTLBPROT	/* DTLB Protection (Read) */
	HANDLER	avr32_wrdtlbprot, AVR32_IRQ_WRDTLBPROT	/* DTLB Protection (Write) */
	HANDLER	avr32_dltbmod, AVR32_IRQ_DLTBMOD		/* DTLB Modified */
	HANDLER	avr32_itlbmiss, AVR32_IRQ_ITLBMISS		/* ITLB Miss */
	HANDLER	avr32_rddtlb, AVR32_IRQ_RDDTLB 			/* DTLB Miss (Read) */
	HANDLER	avr32_wrdtlb, AVR32_IRQ_WRDTLB			/* DTLB Miss (Write) */
	HANDLER	avr32_super, AVR32_IRQ_SUPER			/* Supervisor call */
	HANDLER	avr32_badvector, AVR32_IRQ_BADVECTOR	/* No such vector */

/* Common exception handling logic.  Unlike the interrupt handlers, the
 * exception handlers do not save R8-R12, and LR on the stack.  Only the PC
 * and SR have been pushed onto the system stack by the MCU.  The following
 * loic creates a common stack frame for exception handlers prior to joining
 * to the common interrupt/exception logic below.
 */

avr32_excptcommon:
#warning "Missing Logic"

/****************************************************************************
 * Common Event Handling Logic
 ****************************************************************************/

/* After this point, logic to manage interrupts and exceptions is equivalent.
 * Here we have:
 *
 *   R8-R12, LR, SR, and the PC on the stack.
 *   R12 holds the IRQ number to be dispatched.
 *
 * This function will finish construction of the register save structure and
 * call the IRQ dispatching logic.
 */

avr32_common:
#warning "Missing Logic"
	rjmp	$
	.end


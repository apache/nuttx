/************************************************************
 * up_head.S
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

/************************************************************
 * Included Files
 ************************************************************/

#include <nuttx/irq.h>
#include "up_internal.h"

	.module	up_head
	.optsdcc -mmcs51 --model-large

/************************************************************
 * Private Data
 ************************************************************/

	.area REG_BANK_0	(REL,OVR,DATA)
	.ds 8

/************************************************************
 * Public Data
 ************************************************************/

	.globl	_g_irqtos

/************************************************************
 * Public Functions
 ************************************************************/

	.globl	_irq_dispatch

/************************************************************
 * Program entry points
 ************************************************************/

/* Program entry is through PROGRAM_BASE.  This is just a
 * branch to our start up logic.
 */

	.area	CODE1 (ABS)
	.org	PROGRAM_BASE
	ljmp	start

/* These are indirect interrupt vectors.  Logic in PAULMON2,
 * captures the interrupt vectors (near address 0x0000) and
 * re-routes them through the following entry points.
 *
 * Each of these saves acc and ie then passes the IRQ number
 * to higher level logic in a
 */

	.org	PM2_VECTOR_EXTINT0
	push	acc
	mov	a, #EXT_INT0_IRQ
	ljmp	_up_interrupt

	.org	PM2_VECTOR_TIMER0
	push	acc
	mov	a, #TIMER0_IRQ
	ljmp	_up_interrupt

	.org	PM2_VECTOR_EXTINT1
	push	acc
	mov	a, #EXT_INT1_IRQ
	ljmp	_up_interrupt

	.org	PM2_VECTOR_TIMER1
	push	acc
	mov	a, #TIMER1_IRQ
	ljmp	_up_interrupt

	.org	PM2_VECTOR_UART
	push	acc
	mov	a, #UART_IRQ
	ljmp	_up_interrupt

	.org	PM2_VECTOR_TIMER2
	push	acc
	mov	a, #TIMER2_IRQ
	ljmp	_up_interrupt

/************************************************************
 * Name: start
 *
 * Description:
 *   This is the initial entry point into NuttX
 *
 ************************************************************/

start:
	mov	sp, #(STACK_BASE-1)
	ljmp	_os_start

/************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   All interrupts vector to this point with:
 *
 *   (1) acc on the stack and
 *   (2) the IRQ number in the accumulator
 *
 ************************************************************/

_up_interrupt:
	ar2 = 0x02
	ar3 = 0x03
	ar4 = 0x04
	ar5 = 0x05
	ar6 = 0x06
	ar7 = 0x07
	ar0 = 0x00
	ar1 = 0x01

	/* ACC already on the stack; push IE.  Then disable interrupts */

	push	ie
	clr	ea

	/* Now push the remaining registers with interrupt disabled */

	push	dpl
	push	dph
	push	b
	push	ar2
	push	ar3
	push	ar4
	push	ar5
	push	ar6
	push	ar7
	push	ar0
	push	ar1
	push	psw
	clr	psw
	push	_bp

	/* Mark that we are in an interrupt */

	mov	dptr, #_g_irqtos
        movx    @dptr, a

	/* Now call void irq_dispatch(int irq, FAR void *context)
         *
	 * First, create the first argument as (int)irqno
	 */

        mov     dpl, a
	clr	dph

        /* Create the second argument (void *context) on the stack */

	push	sp
	clr	a
	push	acc

	/* Then dispatch the IRQ.  On return, the stack is off by
	 * by two because the above two pushes, but we fix that by
	 * reloading sp from g_irqtos below.
	 */

	lcall	_irq_dispatch

	/* Get the stackpointer.  This might be the stack pointer that
	 * we increment above or it might be the stack pointer of a
	 * new task if the a context switch happened during the
	 * interrupt handling.
	 */

	mov	dptr, #_g_irqtos
	movx	a, @dptr
	mov	sp, a

	/* Indicate that we are no longer in an interrupt */

	clr	a
	movx	@dptr, a

	/* Then return from the interrupt */

	pop	_bp 
	pop	psw
	pop	ar1 
	pop	ar0 
	pop	ar7 
	pop	ar6 
	pop	ar5 
	pop	ar4 
	pop	ar3 
	pop	ar2 
	pop	b
	pop	dph
	pop	dpl

	/* Restore the interrupt state per the stored IE value */

	pop	acc
	jb	acc.7, 00001$
	clr	ie.7
	sjmp	00002$
  00001$:
	setb	ie.7
  00002$:

	/* Finally pop off the ACC, which was the first register saved. */

	pop acc
	reti

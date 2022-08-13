/****************************************************************************
 * arch/x86_64/src/intel64/intel64_vector.S
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

	.file	"broadwell_vectors.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * .text
 ****************************************************************************/

	.text
    .code64

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.globl	irq_handler
	.globl	isr_handler
	.globl	g_interrupt_stack
	.globl	g_interrupt_stack_end
	.globl	g_isr_stack
	.globl	g_isr_stack_end

/****************************************************************************
 * Macros
 ****************************************************************************/

/* Trace macros, use like trace 'i' to print char to serial port. */

	.macro	trace, ch
	mov		$0x3f8, %dx
	mov		$\ch, %al
	out		%al, %dx
	.endm

/* This macro creates a stub for an ISR which does NOT pass it's own
 * error code (adds a dummy errcode byte).
 */

	.macro ISR_NOERRCODE, intno
	.globl	vector_isr\intno
vector_isr\intno:
	cli							/* Disable interrupts firstly. */

    // CPU has sawitched to the ISR stack using IST

	pushq  $0				/* Push a dummy error code. */

    // Save rdi, rsi
    pushq   %rdi
    pushq   %rsi
	movq   $\intno,   %rsi     /* INT Number is saved to 2nd parameter of function call */
	jmp		isr_common			/* Go to the common ISR handler code. */
	.endm

/* This macro creates a stub for an ISR which passes it's own
 * error code.
 */

	.macro	ISR_ERRCODE, intno
	.globl	vector_isr\intno
vector_isr\intno:
	cli							/* Disable interrupts firstly. */

    // CPU has sawitched to the ISR stack using IST

    // Save rdi, rsi
    pushq   %rdi
    pushq   %rsi
	movq   $\intno,   %rsi     /* INT Number is saved to 2nd parameter of function call */
	jmp		isr_common			/* Go to the common ISR handler code. */
	.endm

/* This macro creates a stub for an IRQ - the first parameter is
 * the IRQ number, the second is the ISR number it is remapped to.
 */

	.macro	IRQ, irqno, intno
	.globl	vector_irq\irqno
vector_irq\irqno:
	cli							/* Disable interrupts firstly. */

    // CPU has switched to the IRQ stack using IST

	pushq  $0				/* Push a dummy error code. */

    // Save rdi, rsi
    pushq   %rdi
    pushq   %rsi
	movq   $\intno,   %rsi     /* INT Number is saved to 2nd parameter of function call */
	jmp		irq_common			/* Go to the common IRQ handler code. */
	.endm

/****************************************************************************
 * IDT Vectors
 ****************************************************************************/
/* The following will be the vector addresses programmed into the IDT */

	ISR_NOERRCODE		ISR0
	.balign 16
	ISR_NOERRCODE		ISR1
	.balign 16
	ISR_NOERRCODE		ISR2
	.balign 16
	ISR_NOERRCODE		ISR3
	.balign 16
	ISR_NOERRCODE		ISR4
	.balign 16
	ISR_NOERRCODE		ISR5
	.balign 16
	ISR_NOERRCODE		ISR6
	.balign 16
	ISR_NOERRCODE		ISR7
	.balign 16
	ISR_ERRCODE			ISR8
	.balign 16
	ISR_NOERRCODE		ISR9
	.balign 16
	ISR_ERRCODE			ISR10
	.balign 16
	ISR_ERRCODE			ISR11
	.balign 16
	ISR_ERRCODE			ISR12
	.balign 16
	ISR_ERRCODE			ISR13
	.balign 16
	ISR_ERRCODE			ISR14
	.balign 16
	ISR_NOERRCODE		ISR15
	.balign 16
	ISR_NOERRCODE		ISR16
	.balign 16
	ISR_NOERRCODE		ISR17
	.balign 16
	ISR_NOERRCODE		ISR18
	.balign 16
	ISR_NOERRCODE		ISR19
	.balign 16
	ISR_NOERRCODE		ISR20
	.balign 16
	ISR_NOERRCODE		ISR21
	.balign 16
	ISR_NOERRCODE		ISR22
	.balign 16
	ISR_NOERRCODE		ISR23
	.balign 16
	ISR_NOERRCODE		ISR24
	.balign 16
	ISR_NOERRCODE		ISR25
	.balign 16
	ISR_NOERRCODE		ISR26
	.balign 16
	ISR_NOERRCODE		ISR27
	.balign 16
	ISR_NOERRCODE		ISR28
	.balign 16
	ISR_NOERRCODE		ISR29
	.balign 16
	ISR_NOERRCODE		ISR30
	.balign 16
	ISR_NOERRCODE		ISR31
	.balign 16
	IRQ				 0,	IRQ0
	.balign 16
	IRQ				 1,	IRQ1
	.balign 16
	IRQ 			 2,	IRQ2
	.balign 16
	IRQ				 3,	IRQ3
	.balign 16
	IRQ				 4,	IRQ4
	.balign 16
	IRQ				 5,	IRQ5
	.balign 16
	IRQ				 6,	IRQ6
	.balign 16
	IRQ				 7,	IRQ7
	.balign 16
	IRQ				 8,	IRQ8
	.balign 16
	IRQ				 9,	IRQ9
	.balign 16
	IRQ				10,	IRQ10
	.balign 16
	IRQ				11,	IRQ11
	.balign 16
	IRQ				12,	IRQ12
	.balign 16
	IRQ				13,	IRQ13
	.balign 16
	IRQ				14,	IRQ14
	.balign 16
	IRQ				15,	IRQ15
	.balign 16

/****************************************************************************
 * Name: isr_common
 *
 * Description:
 *   This is the common ISR logic. It saves the processor state, sets up for
 *   kernel mode segments, calls the C-level fault handler, and finally
 *   restores the stack frame.
 *
 ****************************************************************************/

isr_common:
/*	trace	'S' */
    /* Already swap to the interrupt stack */
    /* stack is automatically recovered by iretq using task state */

    /* x86_64 don't have pusha, we have to do things manually */
    /* RDI and RSI are pushed above for handling IRQ no */
    pushq   %rdx
    pushq   %rcx
    pushq   %r8
    pushq   %r9

    pushq   %r15
    pushq   %r14
    pushq   %r13
    pushq   %r12
    pushq   %r11
    pushq   %r10
    pushq   %rbp
    pushq   %rbx
    pushq   %rax

	mov		%ds, %ax			/* Lower 16-bits of rax. */
	pushq	%rax				/* Save the data segment descriptor */
	mov		%es, %ax			/* Lower 16-bits of rax. */
	pushq	%rax				/* Save the data segment descriptor */
	mov		%gs, %ax			/* Lower 16-bits of rax. */
	pushq	%rax				/* Save the data segment descriptor */
	mov		%fs, %ax			/* Lower 16-bits of rax. */
	pushq	%rax				/* Save the data segment descriptor */

    /* align to 16-bytes boundary */
    leaq -8(%rsp), %rsp

    /* save xmm registers */
    leaq -512(%rsp), %rsp
    fxsaveq (%rsp)

	/* The current value of the SP points to the beginning of the state save
	 * structure.  Save that in RDI as the input parameter to isr_handler.
	 */

	mov		%rsp, %rdi
	call	isr_handler
	jmp		.Lreturn
	.size	isr_common, . - isr_common

/****************************************************************************
 * Name: irq_common
 *
 * Description:
 *   This is the common IRQ logic. It saves the processor state, sets up for
 *   kernel mode segments, calls the C-level fault handler, and finally
 *   restores the stack frame.
 *
 ****************************************************************************/

	.type	irq_common, @function
irq_common:
/*	trace	'R' */
    /* Already swap to the interrupt stack */
    /* stack is automatically recovered by iretq using task state */

    /* x86_64 don't have pusha, we have to do things manually */
    /* RDI and RSI are pushed above for handling IRQ no */
    pushq   %rdx
    pushq   %rcx
    pushq   %r8
    pushq   %r9

    pushq   %r15
    pushq   %r14
    pushq   %r13
    pushq   %r12
    pushq   %r11
    pushq   %r10
    pushq   %rbp
    pushq   %rbx
    pushq   %rax

	mov		%ds, %ax			/* Lower 16-bits of rax. */
	pushq	%rax				/* Save the data segment descriptor */
	mov		%es, %ax			/* Lower 16-bits of rax. */
	pushq	%rax				/* Save the data segment descriptor */
	mov		%gs, %ax			/* Lower 16-bits of rax. */
	pushq	%rax				/* Save the data segment descriptor */
	mov		%fs, %ax			/* Lower 16-bits of rax. */
	pushq	%rax				/* Save the data segment descriptor */

    /* align to 16-bytes boundary */
    leaq -8(%rsp), %rsp

    /* save xmm registers */
    leaq -512(%rsp), %rsp
    fxsaveq (%rsp)

	/* The current value of the SP points to the beginning of the state save
	 * structure.  Save that in RDI as the input parameter to irq_handler.
	 */

	mov		%rsp, %rdi
	call	irq_handler

	/* The common return point for both isr_handler and irq_handler */

.Lreturn:
	/* EAX may possibly hold a pointer to a different register save area on
	 * return.  Are we switching to a new context?
	 */

	cmp		%rax, %rsp
	je		.Lnoswitch

	/* A context swith will be performed. EAX holds the address of the new
	 * register save structure.
	 *
	 * Jump to up_fullcontextrestore().  We perform a call here, but that function
	 * never returns.  The address of the new register save block is the argument
	 * to the up_fullcontextrestore().
	 */

	mov     %rax, %rdi
	call	up_fullcontextrestore

.Lnoswitch:
    fxrstorq (%rsp)
    leaq 512(%rsp), %rsp
    leaq 8(%rsp), %rsp

    popq   %rax
    mov    %fs, %ax
    popq   %rax
    mov    %gs, %ax
    popq   %rax
    mov    %es, %ax
    popq   %rax
    mov    %ds, %ax

    popq   %rax
    popq   %rbx
    popq   %rbp
    popq   %r10
    popq   %r11
    popq   %r12
    popq   %r13
    popq   %r14
    popq   %r15

    popq   %r9
    popq   %r8
    popq   %rcx
    popq   %rdx

    popq   %rsi
    popq   %rdi

	add		$8, %rsp			/* Cleans up the pushed error code */

	iretq						/* Pops 5 things at once: CS, RIP, RFLAGS and SS and RSP */
	.size	irq_common, . - irq_common
	.end


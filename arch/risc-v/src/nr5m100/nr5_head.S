/************************************************************************************
 * arch/risc-v/src/nr5m100/nr5_head.S
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#define ENABLE_QREGS

#include "nr5_custom_ops.h"
#include "nr5_csr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#define NR5M100_STACK_BASE      _ebss
#define NR5M100_STACK_TOP       _ebss+CONFIG_IDLETHREAD_STACKSIZE-4

#if CONFIG_ARCH_INTERRUPTSTACK > 3
#  define NR5M100_INTSTACK_BASE NR5M100_STACK_TOP
#  define NR5M100_INTSTACK_SIZE (CONFIG_ARCH_INTERRUPTSTACK & ~3)
#  define NR5M100_INTSTACK_TOP  NR5M100_STACK_TOP+NR5M100_INTSTACK_SIZE
#  define NR5M100_HEAP_BASE     NR5M100_INTSTACK_TOP
#else
#  define NR5M100_HEAP_BASE     NR5M100_STACK_TOP
#endif

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.global irq_handler

#ifdef CONFIG_RV32IM_HW_MULDIV
	.global time_hard_mul
	.global hard_mul
	.global hard_mulh
	.global hard_mulhsu
	.global hard_div
	.global hard_divu
	.global hard_rem
	.global hard_remu
	.global hard_mulhu
#endif

	.global	g_idle_topstack

	/* Imported symbols */

	.extern __reset_vec
	.extern __trap_vec
	.extern __stack_start
	.global nx_start

#ifndef CONFIG_LIB_NEWLIB

	.section .text
	.global __start

__start:

	/* Set IRQ regs address */

	lui 	x1, %hi(irq_regs)
	addi 	x1, x1, %lo(irq_regs)
	lui 	x2, %hi(irq_regs_addr)
	addi 	x2, x2, %lo(irq_regs_addr)
	sw 		x1, 0(x2)

	/* Set IRQ stack address */

	lui 	x1, %hi(irq_stack)
	addi 	x1, x1, %lo(irq_stack)
	lui 	x2, %hi(irq_stack_addr)
	addi 	x2, x2, %lo(irq_stack_addr)
	sw 		x1, 0(x2)

	/* Zero out the registers */

	addi 	x1, zero, 0
	addi 	x2, zero, 0
	addi 	x3, zero, 0
	addi 	x4, zero, 0
	addi 	x5, zero, 0
	addi 	x6, zero, 0
	addi 	x7, zero, 0
	addi 	x8, zero, 0
	addi 	x9, zero, 0
	addi 	x10, zero, 0
	addi 	x11, zero, 0
	addi 	x12, zero, 0
	addi 	x13, zero, 0
	addi 	x14, zero, 0
	addi 	x15, zero, 0
	addi 	x16, zero, 0
	addi 	x17, zero, 0
	addi 	x18, zero, 0
	addi 	x19, zero, 0
	addi 	x20, zero, 0
	addi 	x21, zero, 0
	addi 	x22, zero, 0
	addi 	x23, zero, 0
	addi 	x24, zero, 0
	addi 	x25, zero, 0
	addi 	x26, zero, 0
	addi 	x27, zero, 0
	addi 	x28, zero, 0
	addi 	x29, zero, 0
	addi 	x30, zero, 0
	addi 	x31, zero, 0

	/* Set stack pointer */

	lui 	sp,%hi(__stack_start)

	/* Initialize global pointer */

1:	auipc 	gp, %pcrel_hi(_gp)
	addi  	gp, gp, %pcrel_lo(1b)

	/* Initialize the Machine Trap Vector */

	lui		t0, %hi(__trap_vec)
	addi	t0, t0, %lo(__trap_vec)
	csrw	CSR_MTVEC, t0

	/* Initialize the Machine Interrupt Table Vector */
	lui     t0, %hi(__reset_vec)
	csrw    CSR_MIVEC, t0

	/* clear the bss segment */

	la		t0, _fbss
	la		t1, _end
1:
#ifdef __riscv64
	sd		zero,0(t0)
	addi	t0, t0, 8
#else
	sw		zero,0(t0)
	addi	t0, t0, 4
#endif
	bltu	t0, t1, 1b

	lw		a0, 0(sp)				# a0 = argc
	addi	a1, sp, _RISCV_SZPTR/8	# a1 = argv
	li		a2, 0					# a2 = envp = NULL

	/* Now jump to the main nr5_init routine to setup interrupts, etc.  */

	la		t0, __nr5_init
	jr		t0

	/* We shouldn't return from __nr5_init */

	.global _init
	.global _fini
_init:
_fini:
	# These don't have to do anything since we use init_array/fini_array.
	ret

#endif

/*
==============================================================
IRQ Handler routine .. save all regs and call the C handler.
==============================================================
*/

irq_handler:

	/* save All registers */

#ifdef ENABLE_QREGS

	/* Save x1,x2 in the spare q2,q3 registers */

	setq 	q2, x1
	setq 	q3, x2

	/* Get pointer to our IRQ REGS save region in RAM */

	lui  	x2, %hi(irq_regs_addr)
	addi 	x2, x2, %lo(irq_regs_addr)
	lw   	x1, 0(x2)
	addi 	x1, x1, 32*4
	sw   	x1, 0(x2)
	addi 	x1, x1, -32*4

	/* Save x3 and x4 to hold regs / stack pointer */

	sw   	x3, 3*4(x1)
	sw   	x4, 4*4(x1)
	addi 	x4, x1, 0

	/* Save the IRQ pending mask to the irq_regs area */

	getq 	x2, q1
	sw   	x2, 33*4(x1)

	/* Save the x1 register (which is in q2) */

	getq 	x2, q2
	sw   	x2, 1*4(x1)

	/* Save the x2 register (which is in q3) */

	getq 	x2, q3
	sw   	x2, 2*4(x1)

	/* Get a stack region in irq stack space */

	lui  	x3, %hi(irq_stack_addr)
	addi 	x3, x3, %lo(irq_stack_addr)
	lw   	x2, 0(x3)
	addi 	x2, x2, -128*4
	sw   	x2, 0(x3)
	addi 	x3, x2, 128*4

	/* Save the return PC.  After the getq of q0,
	 * the higher priority interrupts will be enabled
	 * as we are done using the shared resources (Qregs).
	 */

	getq	x2, q0
	sw		x2,   0*4(x1)

	sw		x5,   5*4(x1)
	sw		x6,   6*4(x1)
	sw		x7,   7*4(x1)
	sw		x8,   8*4(x1)
	sw		x9,   9*4(x1)
	sw		x10, 10*4(x1)
	sw		x11, 11*4(x1)
	sw		x12, 12*4(x1)
	sw		x13, 13*4(x1)
	sw		x14, 14*4(x1)
	sw		x15, 15*4(x1)
	sw		x16, 16*4(x1)
	sw		x17, 17*4(x1)
	sw		x18, 18*4(x1)
	sw		x19, 19*4(x1)
	sw		x20, 20*4(x1)
	sw		x21, 21*4(x1)
	sw		x22, 22*4(x1)
	sw		x23, 23*4(x1)
	sw		x24, 24*4(x1)
	sw		x25, 25*4(x1)
	sw		x26, 26*4(x1)
	sw		x27, 27*4(x1)
	sw		x28, 28*4(x1)
	sw		x29, 29*4(x1)
	sw		x30, 30*4(x1)
	sw		x31, 31*4(x1)

	/* Get the EPIC STATUS */

	csrr	t6, NR5_EPIC_PRIMASK
	sw		t6,  32*4(x1)

	/* Set ISR Stack pointer */

	addi		sp, x3, 0		# IRQ SP is in x3 from above

#else // ENABLE_QREGS

	sw		gp,   0*4+0x200(zero)
	sw		x1,   1*4+0x200(zero)
	sw		x2,   2*4+0x200(zero)
	sw		x3,   3*4+0x200(zero)
	sw		x4,   4*4+0x200(zero)
	sw		x5,   5*4+0x200(zero)
	sw		x6,   6*4+0x200(zero)
	sw		x7,   7*4+0x200(zero)
	sw		x8,   8*4+0x200(zero)
	sw		x9,   9*4+0x200(zero)
	sw		x10, 10*4+0x200(zero)
	sw		x11, 11*4+0x200(zero)
	sw		x12, 12*4+0x200(zero)
	sw		x13, 13*4+0x200(zero)
	sw		x14, 14*4+0x200(zero)
	sw		x15, 15*4+0x200(zero)
	sw		x16, 16*4+0x200(zero)
	sw		x17, 17*4+0x200(zero)
	sw		x18, 18*4+0x200(zero)
	sw		x19, 19*4+0x200(zero)
	sw		x20, 20*4+0x200(zero)
	sw		x21, 21*4+0x200(zero)
	sw		x22, 22*4+0x200(zero)
	sw		x23, 23*4+0x200(zero)
	sw		x24, 24*4+0x200(zero)
	sw		x25, 25*4+0x200(zero)
	sw		x26, 26*4+0x200(zero)
	sw		x27, 27*4+0x200(zero)
	sw		x28, 28*4+0x200(zero)
	sw		x29, 29*4+0x200(zero)
	sw		x30, 30*4+0x200(zero)
	sw		x31, 31*4+0x200(zero)

	/* Get the EPIC STATUS */

	csrr	t6, NR5_EPIC_PRIMASK
	sw		t6,  32*4+0x200(zero)

	/* Set ISR Stack pointer */

	lui 	sp, %hi(irq_stack)
	addi 	sp, sp, %lo(irq_stack)

#endif // ENABLE_QREGS

	/* Set arg0 = address of regs */

#ifdef ENABLE_QREGS
	addi	a0, x4, 0			# REG addr in x4 from above
#else
	lui		a0, %hi(irq_regs)
	addi	a0, a0, %lo(irq_regs)
#endif

  /* Set arg1 = interrupt type */

#ifdef ENABLE_QREGS
	lw		a1, 33*4(x1)
#else
	addi	a1, tp, 0
#endif

	/* call interrupt handler C function */

	jal 	ra, irq_dispatch_all

	/* restore registers */

#ifdef ENABLE_QREGS

	/* new irq_regs address returned from C code in a0 */

	addi 	x1, a0, 0

	lw		x3,   3*4(x1)
	lw		x4,   4*4(x1)
	lw		x5,   5*4(x1)
	lw		x6,   6*4(x1)
	lw		x7,   7*4(x1)
	lw		x8,   8*4(x1)
	lw		x9,   9*4(x1)
	lw		x10, 10*4(x1)
	lw		x11, 11*4(x1)
	lw		x12, 12*4(x1)
	lw		x13, 13*4(x1)
	lw		x14, 14*4(x1)
	lw		x15, 15*4(x1)
	lw		x16, 16*4(x1)
	lw		x17, 17*4(x1)
	lw		x18, 18*4(x1)
	lw		x19, 19*4(x1)
	lw		x20, 20*4(x1)
	lw		x21, 21*4(x1)
	lw		x22, 22*4(x1)
	lw		x23, 23*4(x1)
	lw		x24, 24*4(x1)
	lw		x25, 25*4(x1)
	lw		x26, 26*4(x1)
	lw		x27, 27*4(x1)
	lw		x28, 28*4(x1)
	lw		x29, 29*4(x1)
	lw		x30, 30*4(x1)
	lw		x31, 31*4(x1)

	/* Restore return PC to q0.  This will cause the
	 * processor to enter a critical state where
	 * higher priority IRQs won't happen until after
	 * the mret, thus protecting our shared QREGS.
	 */

	lw		x2, 0*4(x1)
	setq	q0, x2

	/* Restore EPIC STATUS (it may have changed) */

	lw		x2, 32*4(x1)
	csrw	NR5_EPIC_PRIMASK, x2

	/* Restore the IRQ mask so the processor knows
	 * which interrupts to signal as handled
	 */

	lw		x2, 33*4(x1)
	setq	q1, x2

	/* Put original x1 into q2 */

	lw		x2, 1*4(x1)
	setq	q2, x2

	/* Put original x2 into q3 */

	lw		x2, 2*4(x1)
	setq	q3, x2

	/* Deallocate the irq_stack space */

	lui		x2, %hi(irq_stack_addr)
	addi	x2, x2, %lo(irq_stack_addr)
	lw		x1, 0(x2)
	addi	x1, x1, 128*4
	sw		x1, 0(x2)

	/* Deallocate the irq_regs space */

	lui		x2, %hi(irq_regs_addr)
	addi	x2, x2, %lo(irq_regs_addr)
	lw		x1, 0(x2)
	addi	x1, x1, -32*4
	sw		x1, 0(x2)

	/* Now restore original x1,x2 from q2,q3 */

	getq	x1, q2
	getq	x2, q3

#else // ENABLE_QREGS

	/* new irq_regs address returned from C code in a0 */

	addi	a1, zero, 0x200
	beq		a0, a1, 1f
	sbreak
1:

	lw		gp,   0*4+0x200(zero)
	lw		x1,   1*4+0x200(zero)
	lw		x2,   2*4+0x200(zero)
	// do not restore x3 (gp)
	lw		x4,   4*4+0x200(zero)
	lw		x5,   5*4+0x200(zero)
	lw		x6,   6*4+0x200(zero)
	lw		x7,   7*4+0x200(zero)
	lw		x8,   8*4+0x200(zero)
	lw		x9,   9*4+0x200(zero)
	lw		x10, 10*4+0x200(zero)
	lw		x11, 11*4+0x200(zero)
	lw		x12, 12*4+0x200(zero)
	lw		x13, 13*4+0x200(zero)
	lw		x14, 14*4+0x200(zero)
	lw		x15, 15*4+0x200(zero)
	lw		x16, 16*4+0x200(zero)
	lw		x17, 17*4+0x200(zero)
	lw		x18, 18*4+0x200(zero)
	lw		x19, 19*4+0x200(zero)
	lw		x20, 20*4+0x200(zero)
	lw		x21, 21*4+0x200(zero)
	lw		x22, 22*4+0x200(zero)
	lw		x23, 23*4+0x200(zero)
	lw		x24, 24*4+0x200(zero)
	lw		x25, 25*4+0x200(zero)
	lw		x26, 26*4+0x200(zero)
	lw		x27, 27*4+0x200(zero)
	lw		x28, 28*4+0x200(zero)
	lw		x29, 29*4+0x200(zero)
	lw		x30, 30*4+0x200(zero)
	lw		x31, 31*4+0x200(zero)

#endif // ENABLE_QREGS

	/* Return from Machine Interrupt */

    mret

irq_regs:
	/* registers are saved to this memory region during interrupt handling
	 * the program counter is saved as register 0
	 */

	.fill	34*5,4

	/* stack for the interrupt handler */

	.fill	128*5,4
irq_stack:

irq_regs_addr:
   .fill	4,4
irq_stack_addr:
   .fill	4,4


/****************************************************************************
 * Hard mul and div functions for multest.  These are C interfaces to
 * the MUL / DIV opcodes for performing HARD vs SOFT testing.
 ****************************************************************************/

#ifdef CONFIG_RV32IM_HW_MULDIV

/* Multiply, return lower 32 bits */

up_hard_mul:
	mul		a0, a0, a1
	ret

/* Multiply, return upper 32 bits */

up_hard_mulh:
	mulh	a0, a0, a1
	ret

/* Multiply unsigned */

up_hard_mulhsu:
	mulhsu	a0, a0, a1
	ret

up_hard_mulhu:
	mulhu	a0, a0, a1
	ret

/* Divide */

up_hard_div:
	div		a0, a0, a1
	ret

/* Divide, return remainder */

up_hard_rem:
	rem		a0, a0, a1
	ret

/* Divide, unsigned */

up_hard_divu:
	divu	a0, a0, a1
	ret

/* Remainder, unsigned */

up_hard_remu:
	remu	a0, a0, a1
	ret

/* Calculate number of clock cycles required for MUL */

up_time_hard_mul:
	rdcycle	a3
	mul		a0, a0, a1
	rdcycle	a4
	sub		a1, a4, a3
	sw		a1,0(a2)
	ret

#endif

/****************************************************************************
 * This global variable is unsigned int g_idle_topstack and is exported here only
 * because of its coupling to idle thread stack.
 ****************************************************************************/

	.section .data
	.type	g_idle_topstack, object
g_idle_topstack:
	.long	NR5M100_HEAP_BASE
	.size	g_idle_topstack, .-g_idle_topstack

/****************************************************************************
 * Modeline to set vim formatting options for ASM file.  For this to work,
 * you must enable moeline processing in your ~/.vimrc file with:
 *
 *    ~/.vimrc:
 *      set modeline
 *
 * vim: noet:ts=4:sw=4
 ****************************************************************************/

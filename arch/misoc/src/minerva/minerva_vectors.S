/****************************************************************************
 * arch/misoc/src/minerva/minerva_vectors.S
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MINERVA_STACK_BASE    _ebss
#define MINERVA_STACK_TOP     _ebss+CONFIG_IDLETHREAD_STACKSIZE
#define MINERVA_HEAP_BASE     MINERVA_STACK_TOP

#define MIE_MEIE              0x800
#define CSR_IRQ_MASK          0x330

/****************************************************************************
 * Macros
 ****************************************************************************/

	.section	.text, "ax", @progbits
	.global		g_idle_topstack
	.global		__start

.macro SAVE_REGS
	addi sp, sp, -XCPTCONTEXT_SIZE
	sw x0, REG_X0(sp)
	sw x1, REG_X1(sp)
	sw x3, REG_X3(sp)
	sw x4, REG_X4(sp)
	sw x5, REG_X5(sp)
	sw x6, REG_X6(sp)
	sw x7, REG_X7(sp)
	sw x8, REG_X8(sp)
	sw x9, REG_X9(sp)
	sw x10, REG_X10(sp)
	sw x11, REG_X11(sp)
	sw x12, REG_X12(sp)
	sw x13, REG_X13(sp)
	sw x14, REG_X14(sp)
	sw x15, REG_X15(sp)
	sw x16, REG_X16(sp)
	sw x17, REG_X17(sp)
	sw x18, REG_X18(sp)
	sw x19, REG_X19(sp)
	sw x20, REG_X20(sp)
	sw x21, REG_X21(sp)
	sw x22, REG_X22(sp)
	sw x23, REG_X23(sp)
	sw x24, REG_X24(sp)
	sw x25, REG_X25(sp)
	sw x26, REG_X26(sp)
	sw x27, REG_X27(sp)
	sw x28, REG_X28(sp)
	sw x29, REG_X29(sp)
	sw x30, REG_X30(sp)
	sw x31, REG_X31(sp)
	csrr t0, mstatus
	csrr t1, mepc
	csrr t2, mbadaddr
	csrr t3, mcause
	sw t0, REG_CSR_MSTATUS(sp)
	sw t1, REG_CSR_MEPC(sp)
	sw t2, REG_CSR_MBADADDR(sp)
	sw t3, REG_CSR_MCAUSE(sp)
	addi t0, sp, XCPTCONTEXT_SIZE
	sw t0, REG_X2(sp)
 .endm

/* restore regs and `mret` */

 .macro RESTORE_REGS
	lw x1, REG_X1(a0)
	lw x2, REG_X2(a0)
	lw x3, REG_X3(a0)
	lw x4, REG_X4(a0)
	lw x6, REG_X6(a0)
	lw x7, REG_X7(a0)
	lw x8, REG_X8(a0)
	lw x9, REG_X9(a0)
	lw x11, REG_X11(a0)
	lw x12, REG_X12(a0)
	lw x13, REG_X13(a0)
	lw x14, REG_X14(a0)
	lw x15, REG_X15(a0)
	lw x16, REG_X16(a0)
	lw x17, REG_X17(a0)
	lw x18, REG_X18(a0)
	lw x19, REG_X19(a0)
	lw x20, REG_X20(a0)
	lw x21, REG_X21(a0)
	lw x22, REG_X22(a0)
	lw x23, REG_X23(a0)
	lw x24, REG_X24(a0)
	lw x25, REG_X25(a0)
	lw x26, REG_X26(a0)
	lw x27, REG_X27(a0)
	lw x28, REG_X28(a0)
	lw x29, REG_X29(a0)
	lw x30, REG_X30(a0)
	lw x31, REG_X31(a0)

	lw t0, REG_CSR_MEPC(a0)
	csrw mepc, t0

	lw t0, REG_CSR_MSTATUS(a0)
	csrw mstatus, t0

	lw x5, REG_X5(a0)
	lw x10, REG_X10(a0)
 .endm

/****************************************************************************
 * Reset Handler
 ****************************************************************************/

__start:
_reset_handler:
	/* Set stack pointer */

	la sp, _fstack
	la t0, trap_entry
	csrw mtvec, t0

	/* Initialize .bss */

	la t0, _sbss
	la t1, _ebss
1:	beq t0, t1, 2f
	sw zero, 0(t0)
	addi t0, t0, 4
	j 1b
2:
#if 0 /* REVISIT */
	/* Enable external interrupts */

	li t0, MIE_MEIE
	csrs mie, t0
#endif

	call nx_start
1:	j 1b

trap_entry:
	SAVE_REGS
	mv a0, sp
	call minerva_doexception
	RESTORE_REGS
	mret
	nop
	nop
	nop
	nop

	/* This global variable is unsigned long g_idle_topstack and is
	 * exported from here only because of its coupling to other
	 * uses of _ebss in this file
	 */

	.data
	.align  4
	.type   g_idle_topstack, object

g_idle_topstack:
	.long	MINERVA_STACK_TOP
	.size	g_idle_topstack, .-g_idle_topstack
	.end

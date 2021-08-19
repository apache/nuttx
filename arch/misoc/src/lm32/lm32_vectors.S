/****************************************************************************
 * arch/misoc/src/lm32/lm32_allocateheap.c
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
/* Linker memory organization ***********************************************/
/* Data memory is organized as follows:
 *
 * 1) Initialized data (.data):
 *    Start:   _sdata
 *    End(+1): _edata
 * 2) Uninitialized data (.bss):
 *    Start:   _sbss
 *    End(+1): _ebss
 *
 * The following are placed outside of the "normal" memory segments -- mostly
 * so that they do not have to be cleared on power up.
 *
 * 3) Idle thread stack:
 *    Start:   _ebss
 *    End(+1): _ebss+CONFIG_IDLETHREAD_STACKSIZE
 * 4) Heap:
 *    Start:   _ebss+CONFIG_IDLETHREAD_STACKSIZE
 *    End(+1): to the end of memory
 */

#define LM32_STACK_BASE    _ebss
#define LM32_STACK_TOP     _ebss+CONFIG_IDLETHREAD_STACKSIZE
#define LM32_HEAP_BASE     LM32_STACK_TOP

/****************************************************************************
 * Exception handlers - Must be 32 bytes long.
 ****************************************************************************/

	.section	.text, "ax", @progbits
	.global		g_idle_topstack
	.global		__start

__start:
_reset_handler:
	xor		r0, r0, r0
	wcsr	IE, r0
	mvhi	r1, hi(_reset_handler)
	ori		r1, r1, lo(_reset_handler)
	wcsr	EBA, r1
	bi		_do_reset
	nop
	nop

_breakpoint_handler:
	bi		_breakpoint_handler
	nop
	nop
	nop
	nop
	nop
	nop
	nop

_instruction_bus_error_handler:
	bi		_instruction_bus_error_handler
	nop
	nop
	nop
	nop
	nop
	nop
	nop

_watchpoint_hander:
	bi		_watchpoint_hander
	nop
	nop
	nop
	nop
	nop
	nop
	nop

_data_bus_error_handler:
	bi		_data_bus_error_handler
	nop
	nop
	nop
	nop
	nop
	nop
	nop

_divide_by_zero_handler:
	bi		_divide_by_zero_handler
	nop
	nop
	nop
	nop
	nop
	nop
	nop

_interrupt_handler:
	sw		(sp+0), ra
	calli	.save_all
	rcsr	r1, IP
	calli	lm32_decodeirq
	bi		.restore_all_and_eret
	nop
	nop
	nop

_syscall_handler:
	sw		(sp+0), ra
	addi	ea, ea, 4
	calli	.save_all
	mvi		r1, LM32_IRQ_SWINT
	calli	lm32_doirq
	bi		.restore_all_and_eret
	nop
	nop

_do_reset:
	/* Setup stack and global pointer */

	mvhi	sp, hi(LM32_STACK_TOP)
	ori		sp, sp, lo(LM32_STACK_TOP)

	/* Clear BSS */

	mvhi	r1, hi(_sbss)
	ori		r1, r1, lo(_sbss)
	mvhi	r3, hi(_ebss)
	ori		r3, r3, lo(_ebss)

.clearBSS:
	be		r1, r3, .callMain
	sw		(r1+0), r0
	addi	r1, r1, 4
	bi		.clearBSS

.callMain:
	bi		nx_start

.save_all:
	addi	sp, sp, -136
	sw		(sp+REG_X0), r0
	sw		(sp+REG_X1), r1
	sw		(sp+REG_X2), r2
	sw		(sp+REG_X3), r3
	sw		(sp+REG_X4), r4
	sw		(sp+REG_X5), r5
	sw		(sp+REG_X6), r6
	sw		(sp+REG_X7), r7
	sw		(sp+REG_X8), r8
	sw		(sp+REG_X9), r9
	sw		(sp+REG_X10), r10
	sw		(sp+REG_X11), r11
	sw		(sp+REG_X12), r12
	sw		(sp+REG_X13), r13
	sw		(sp+REG_X14), r14
	sw		(sp+REG_X15), r15
	sw		(sp+REG_X16), r16
	sw		(sp+REG_X17), r17
	sw		(sp+REG_X18), r18
	sw		(sp+REG_X19), r19
	sw		(sp+REG_X20), r20
	sw		(sp+REG_X21), r21
	sw		(sp+REG_X22), r22
	sw		(sp+REG_X23), r23
	sw		(sp+REG_X24), r24
	sw		(sp+REG_X25), r25
	sw		(sp+REG_GP), r26
	sw		(sp+REG_FP), r27

	/* Save SP before we add 136 */

	addi	r1, sp, 136
	sw		(sp+REG_SP), r1

	/* Reg RA done later */

	sw		(sp+REG_EA), r30
	sw		(sp+REG_BA), r31

	/* ra needs to be moved from initial stack location */

	lw		r1, (sp+ 136)
	sw		(sp+REG_RA), r1

	/* Get IE/REG_INT_CTX */

	rcsr	r1, IE
	sw		(sp+REG_INT_CTX), r1

	/* The 2nd argument is the regs pointer */

	addi	r2, sp, 0

	/* Move sp away from X0 */

	addi	sp, sp, -4
	ret

.restore_all_and_eret:
	/* r1 should have the place where we restore ! */

	lw		r3, (r1+REG_X3)
	lw		r4, (r1+REG_X4)
	lw		r5, (r1+REG_X5)
	lw		r6, (r1+REG_X6)
	lw		r7, (r1+REG_X7)
	lw		r8, (r1+REG_X8)
	lw		r9, (r1+REG_X9)
	lw		r10, (r1+REG_X10)
	lw		r11, (r1+REG_X11)
	lw		r12, (r1+REG_X12)
	lw		r13, (r1+REG_X13)
	lw		r14, (r1+REG_X14)
	lw		r15, (r1+REG_X15)
	lw		r16, (r1+REG_X16)
	lw		r17, (r1+REG_X17)
	lw		r18, (r1+REG_X18)
	lw		r19, (r1+REG_X19)
	lw		r20, (r1+REG_X20)
	lw		r21, (r1+REG_X21)
	lw		r22, (r1+REG_X22)
	lw		r23, (r1+REG_X23)
	lw		r24, (r1+REG_X24)
	lw		r25, (r1+REG_X25)
	lw		r26, (r1+REG_GP)
	lw		r27, (r1+REG_FP)
	lw		r28, (r1+REG_SP)
	lw		r29, (r1+REG_RA)
	lw		r30, (r1+REG_EA)
	lw		r31, (r1+REG_BA)
	lw		r2, (r1+REG_INT_CTX)
	wcsr	IE, r2
	lw		r2, (r1+REG_X2)
	lw		r1, (r1+REG_X1)

	eret

	/* This global variable is unsigned long g_idle_topstack and is
	 * exported from here only because of its coupling to other
	 * uses of _ebss in this file
	 */

	.data
	.align	4
	.type	g_idle_topstack, object

g_idle_topstack:
	.long	LM32_STACK_TOP
	.size	g_idle_topstack, .-g_idle_topstack
	.end

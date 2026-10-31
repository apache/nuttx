/****************************************************************************
 * arch/rh850/src/common/rh850_exception_common.S
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include "rh850_internal.h"
#include "rh850_macros.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* TRAP0 (system call): EIPC = next instruction, PSW.ID = EP = 1.  EIRET
 * with EP = 1 leaves ISPR unchanged.
 */

	.text
	.globl	_rh850_trap0
	.type	_rh850_trap0, @function
_rh850_trap0:
	rh850_frame_save
	mov	sp, r6
	jarl	_rh850_doswi, lp
	mov	r10, sp
	rh850_frame_restore
	.size	_rh850_trap0, . - _rh850_trap0

/* EIINT, all priority levels: irq = EIIC - 1000H.  EIRET with EP = 0
 * clears the ISPR bit of the request.
 */

	.globl	_rh850_eiint
	.type	_rh850_eiint, @function
_rh850_eiint:
	rh850_frame_save
	mov	sp, r20
	mov	hilo(_g_interrupt_context), r10
	ld.bu	0[r10], r10
	cmp	0, r10
	bne	1f
	mov	hilo(_g_intstacktop), sp
1:
	rh850_stsr	RH850_SR_EIIC, r6
	andi	0xffff, r6, r6
	addi	-0x1000, r6, r6
	mov	r20, r7
	jarl	_rh850_doirq, lp
	mov	r10, sp
	rh850_frame_restore
	.size	_rh850_eiint, . - _rh850_eiint

/* CPU exceptions: rh850_exception(cause, level, regs) on the interrupt
 * stack.  It does not return.
 */

	.globl	_rh850_exc_fe
	.type	_rh850_exc_fe, @function
_rh850_exc_fe:
	rh850_frame_save fe=1, exc=1
	rh850_stsr	RH850_SR_FEIC, r6
	mov	RH850_EXC_LEVEL_FE, r7
	br	1f
	.size	_rh850_exc_fe, . - _rh850_exc_fe

	.globl	_rh850_exc_ei
	.type	_rh850_exc_ei, @function
_rh850_exc_ei:
	rh850_frame_save exc=1
	rh850_stsr	RH850_SR_EIIC, r6
	mov	RH850_EXC_LEVEL_EI, r7
1:
	mov	sp, r8
	mov	hilo(_g_interrupt_context), r10
	ld.bu	0[r10], r10
	cmp	0, r10
	bne	2f
	mov	hilo(_g_intstacktop), sp
2:
	jarl	_rh850_exception, lp
	di
3:
	halt
	br	3b
	.size	_rh850_exc_ei, . - _rh850_exc_ei

/****************************************************************************
 * Interrupt Stack
 ****************************************************************************/

	.section .bss
	.balign	4
	.globl	_g_intstackalloc
	.globl	_g_intstacktop
_g_intstackalloc:
	.skip	(CONFIG_ARCH_INTERRUPTSTACK & ~3)
_g_intstacktop:

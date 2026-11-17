/****************************************************************************
 * arch/tricore/src/common/gnu/tricore_vectors.S
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
 * Trap vector table (BTV) in .traptab and interrupt vector table (BIV) in
 * .inttab; the linker script places both sections.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

.macro	TRAPSLOT cls, entry
	svlcx
	mov	%d5, \cls
	j	\entry
	.p2align 5
.endm

.macro	INTSLOT handler
	svlcx
	jl	\handler
	rslcx
	rfe
	.p2align 5
.endm

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Tricore_trap_entry
 ****************************************************************************/

	.text
	.p2align 1

	.global	tricore_trap_entry
	.type	tricore_trap_entry, @function
tricore_trap_entry:
	mov.aa	%a4, %a11
	mov	%d4, %d15
	jl	tricore_trapcall
	rslcx
	rfe
	.size	tricore_trap_entry, . - tricore_trap_entry

/****************************************************************************
 * Tricore_syscall_entry
 ****************************************************************************/

	.type	tricore_syscall_entry, @function
tricore_syscall_entry:
	jl	tricore_svcall
	rslcx
	rfe
	.size	tricore_syscall_entry, . - tricore_syscall_entry

/****************************************************************************
 * Tricore_fcu_entry
 ****************************************************************************/

	.type	tricore_fcu_entry, @function
tricore_fcu_entry:
	movh.a	%a4, hi:g_fcu_csa_base
	ld.a	%a4, [%a4]lo:g_fcu_csa_base
.Lfcu_nopool:
	jz.a	%a4, .Lfcu_nopool
	stucx	[%a4]0
	stlcx	[%a4]64
	jl	tricore_fcutrap_recover
	movh.a	%a4, hi:g_fcu_csa_base
	ld.a	%a4, [%a4]lo:g_fcu_csa_base
	ld.a	%a11, [%a4](REG_UA11 * 4)
	mov	%d15, 4
	mov	%d5, 3
	j	tricore_trap_entry
	.size	tricore_fcu_entry, . - tricore_fcu_entry

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Trap vector table
 ****************************************************************************/

	.section .traptab, "ax", @progbits
	.p2align 8
	.global	tricore_traptab
	.type	tricore_traptab, @function
tricore_traptab:
	TRAPSLOT 0, tricore_trap_entry
	TRAPSLOT 1, tricore_trap_entry
	TRAPSLOT 2, tricore_trap_entry

	jne	%d15, 4, 1f
	j	tricore_fcu_entry
1:
	TRAPSLOT 3, tricore_trap_entry

	TRAPSLOT 4, tricore_trap_entry
	TRAPSLOT 5, tricore_trap_entry
	TRAPSLOT 6, tricore_syscall_entry
	TRAPSLOT 7, tricore_trap_entry
	.size	tricore_traptab, . - tricore_traptab

/****************************************************************************
 * Interrupt vector table.  One entry serves all priorities; the startup
 * code sets the BIV priority bits accordingly.
 ****************************************************************************/

	.section .inttab, "ax", @progbits
	.p2align 5
	.global	tricore_inttab
	.type	tricore_inttab, @function
tricore_inttab:
	INTSLOT	tricore_doirq
	.size	tricore_inttab, . - tricore_inttab

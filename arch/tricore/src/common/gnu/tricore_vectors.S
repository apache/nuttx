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
 * Trap vector table, one per core.  BTV points at the table of the core in
 * use; the linker script keeps the .traptab_cpuN section of that core.
 *
 * Each 32-byte slot saves the lower context and enters the common trap
 * entry with the trap class in %d5; the hardware has put the TIN in %d15
 * and the trap return address in %a11.  The entry builds a struct
 * tricore_trap_s on the stack and enters the handler with jli, so the
 * handler sees PCXI pointing at the lower context of the trapped code.
 * The handler returns with ji %a11, then the entry restores the lower
 * context and returns from the trap.
 *
 * Context management traps go through tricore_ctx_entry first: on a free
 * context list underflow (FCU, TIN 4) there is no free CSA, so svlcx
 * cannot run.  tricore_fcu_entry then saves the trapped context into the
 * pool registered with tricore_fcu_register_pool() and continues in
 * tricore_fcu_recover().  The saved %a4 is lost, as the entry uses it, and
 * so is %sp when the trap switched to the interrupt stack.  Without a pool
 * the core stops in tricore_fcu_entry.
 *
 * tasking/tricore_vectors.asm is the same code in the TASKING syntax.
 ****************************************************************************/

#include <arch/csfr.h>

	.macro	TRAPSLOT cls, entry
	svlcx
	mov	%d5, \cls
	j	\entry
	.p2align 5
	.endm

	.macro	TRAPJUMP entry
	j	\entry
	.p2align 5
	.endm

	.macro	TRAPTAB cpu
	.section .traptab_cpu\cpu, "ax", %progbits
	.p2align 8
	.global	tricore_trap_vectors_cpu\cpu
tricore_trap_vectors_cpu\cpu:
	TRAPSLOT 0, tricore_trap_entry
	TRAPSLOT 1, tricore_trap_entry
	TRAPSLOT 2, tricore_trap_entry
	TRAPJUMP tricore_ctx_entry
	TRAPSLOT 4, tricore_trap_entry
	TRAPSLOT 5, tricore_trap_entry
	TRAPSLOT 6, tricore_syscall_entry
	TRAPSLOT 7, tricore_trap_entry
	.endm

	TRAPTAB 0
	TRAPTAB 1
	TRAPTAB 2
	TRAPTAB 3
	TRAPTAB 4
	TRAPTAB 5

	/* struct tricore_trap_s: taddr at 0, tid at 4, tclass at 5, tcpu at 6 */

	.macro	TRAPENTRY name, handler
	.global	\name
	.type	\name, %function
\name:
	sub.a	%sp, 8
	st.a	[%sp]0, %a11
	st.b	[%sp]4, %d15
	st.b	[%sp]5, %d5
	mfcr	%d4, TRICORE_CPU_CORE_ID
	and	%d4, %d4, 7
	st.b	[%sp]6, %d4
	mov.aa	%a4, %sp
	movh.a	%a2, hi:\handler
	lea	%a2, [%a2]lo:\handler
	jli	%a2
	lea	%sp, [%sp]8
	rslcx
	rfe
	.endm

	.text
	.p2align 1

	TRAPENTRY tricore_trap_entry, tricore_trapcall
	TRAPENTRY tricore_syscall_entry, tricore_svcall

	.type	tricore_ctx_entry, %function
tricore_ctx_entry:
	jeq	%d15, 4, tricore_fcu_entry
	svlcx
	mov	%d5, 3
	j	tricore_trap_entry

	.type	tricore_fcu_entry, %function
tricore_fcu_entry:
	movh.a	%a4, hi:g_tricore_fcu_pool
	ld.a	%a4, [%a4]lo:g_tricore_fcu_pool
.Lfcu_nopool:
	jz.a	%a4, .Lfcu_nopool
	stucx	[%a4]0
	stlcx	[%a4]64
	j	tricore_fcu_recover

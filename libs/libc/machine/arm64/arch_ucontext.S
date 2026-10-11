/****************************************************************************
 * libs/libc/machine/arm64/arch_ucontext.S
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Offsets in mcontext_t (arch/arm64/include/ucontext.h), which is at the
 * start of ucontext_t.
 */

#define UC_X0           0
#define UC_X19          (19 * 8)
#define UC_SP           (31 * 8)
#define UC_PC           (32 * 8)
#define UC_D8           (33 * 8)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

	.text

/* Save the callee-saved registers of the caller in the context at \base:
 * the context resumes at the return address with x0 = 0.
 */

	.macro	save_context base
	stp	x19, x20, [\base, #UC_X19]
	stp	x21, x22, [\base, #UC_X19 + 16]
	stp	x23, x24, [\base, #UC_X19 + 32]
	stp	x25, x26, [\base, #UC_X19 + 48]
	stp	x27, x28, [\base, #UC_X19 + 64]
	stp	x29, x30, [\base, #UC_X19 + 80]
	str	xzr, [\base, #UC_X0]
	mov	x9, sp
	stp	x9, x30, [\base, #UC_SP]
#ifdef CONFIG_ARCH_FPU
	stp	d8, d9, [\base, #UC_D8]
	stp	d10, d11, [\base, #UC_D8 + 16]
	stp	d12, d13, [\base, #UC_D8 + 32]
	stp	d14, d15, [\base, #UC_D8 + 48]
#endif
	.endm

/* int getcontext(ucontext_t *ucp) */

	.global	getcontext
	.type	getcontext, %function
getcontext:
	save_context x0
	b	arm64_ucontext_getmask
	.size	getcontext, . - getcontext

/* int setcontext(const ucontext_t *ucp) */

	.global	setcontext
	.type	setcontext, %function
setcontext:
	mov	x19, x0
	bl	arm64_ucontext_setmask
	mov	x16, x19

/* Load the context at x16 and jump to its pc */

arm64_ucontext_load:
#ifdef CONFIG_ARCH_FPU
	ldp	d8, d9, [x16, #UC_D8]
	ldp	d10, d11, [x16, #UC_D8 + 16]
	ldp	d12, d13, [x16, #UC_D8 + 32]
	ldp	d14, d15, [x16, #UC_D8 + 48]
#endif
	ldp	x19, x20, [x16, #UC_X19]
	ldp	x21, x22, [x16, #UC_X19 + 16]
	ldp	x23, x24, [x16, #UC_X19 + 32]
	ldp	x25, x26, [x16, #UC_X19 + 48]
	ldp	x27, x28, [x16, #UC_X19 + 64]
	ldp	x29, x30, [x16, #UC_X19 + 80]
	ldp	x2, x3, [x16, #UC_X0 + 16]
	ldp	x4, x5, [x16, #UC_X0 + 32]
	ldp	x6, x7, [x16, #UC_X0 + 48]
	ldp	x17, x1, [x16, #UC_SP]
	mov	sp, x17
	mov	x17, x1
	ldp	x0, x1, [x16, #UC_X0]
	br	x17
	.size	setcontext, . - setcontext

/* int swapcontext(ucontext_t *oucp, const ucontext_t *ucp) */

	.global	swapcontext
	.type	swapcontext, %function
swapcontext:
	save_context x0
	mov	x19, x0
	mov	x20, x1
	mov	x0, x20
	mov	x1, x19
	bl	arm64_ucontext_swapmask
	mov	x16, x20
	b	arm64_ucontext_load
	.size	swapcontext, . - swapcontext

/* A context made by makecontext() returns here: x19 holds its uc_link */

	.global	arm64_ucontext_start
	.type	arm64_ucontext_start, %function
arm64_ucontext_start:
	mov	x0, x19
	cbz	x0, 1f
	bl	setcontext
1:
	mov	x0, #0
	bl	exit
	.size	arm64_ucontext_start, . - arm64_ucontext_start
